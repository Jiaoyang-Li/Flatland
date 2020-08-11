#include "MCP.h"

// TODO: if the agent is already appeared on the map,
//  but because of the delays, its replanned path deos not reach its goal location,
//  so we cannot assume that the last location in an agent's path is its goal location
void MCP::simulate(vector<Path>& paths, int timestep) const
{
    auto copy_agent_time = agent_time;
    auto copy_mcp = mcp;
    paths.resize(al->paths_all.size());
    list<int> unfinished_agents;
    for (int i = 0; i < al->getNumOfAllAgents(); i++)
    {
        if (al->agents_all[i].status >= 2 || // the agent has already reached its goal location
            al->paths_all[i].empty()) // the agent did not have a path
            continue;
        unfinished_agents.push_back(i);
        paths.reserve(al->paths_all[i].size() * 2);
        if (copy_agent_time[i] > 0)
        {
            assert(copy_agent_time[i] < (int)no_wait_time[i].size());
            int curr = ml->linearize_coordinate(al->agents_all[i].position);
            if (al->paths_all[i][no_wait_time[i][copy_agent_time[i] - 1]].location != curr)
            {
                cout << curr << "," << al->paths_all[i][no_wait_time[i][copy_agent_time[i] - 1]].location << endl;
                cout << "BOOM" << endl;
            }
            assert(al->paths_all[i][no_wait_time[i][copy_agent_time[i] - 1]].location ==
                ml->linearize_coordinate(al->agents_all[i].position));
            paths[i].push_back(al->paths_all[i][no_wait_time[i][copy_agent_time[i] - 1]]);
        }
        else
        {
            assert(al->agents_all[i].status == 0);
            paths[i].push_back(al->paths_all[i].front());
        }
    }
    for (int t = 0; t < al->constraintTable.length_max && !unfinished_agents.empty(); t++) {
        for (auto p = unfinished_agents.begin(); p != unfinished_agents.end();) {
            int i = *p;
            assert(copy_agent_time[i] <= (int) no_wait_time[i].size());
            if (copy_agent_time[i] == (int) no_wait_time[i].size())  // the agent has reached its location
            {
                int previous = al->paths_all[i][no_wait_time[i][copy_agent_time[i] - 1]].location;
                assert(get<0>(copy_mcp[previous].front()) == i);
                copy_mcp[previous].pop_front();
                p = unfinished_agents.erase(p);
                continue;
            }
            if (appear_time[i] > t + 1 + timestep) // the agent should not appear before the appear time
            {
                paths[i].push_back(paths[i].back()); // stay still
            }
            else if (al->agents_all[i].malfunction_left > t + 1 ||
                (al->agents_all[i].malfunction_left > t && copy_agent_time[i] > 0)) // the agent is still in mal
            { // mal function does not affect the action of appearing at the start location
                // therefore, the agent can appear at the start location when malfunction_left - t = 1
                paths[i].push_back(paths[i].back()); // stay still
            }
            else // check mcp to determine whether the agent should move or wait
            {
                int loc = al->paths_all[i][no_wait_time[i][copy_agent_time[i]]].location;
                assert(!copy_mcp[loc].empty());
                if (get<0>(copy_mcp[loc].front()) == i) {
                    paths[i].push_back(al->paths_all[i][no_wait_time[i][copy_agent_time[i]]]); // move
                    if ( copy_agent_time[i] > 0)
                    {
                        int previous = al->paths_all[i][no_wait_time[i][copy_agent_time[i] - 1]].location;
                        assert(get<0>(copy_mcp[previous].front()) == i);
                        copy_mcp[previous].pop_front();
                    }
                    copy_agent_time[i]++;
                } else {
                    paths[i].push_back(paths[i].back()); // stay still
                }
            }
            ++p; // next agent
        }
    }
}


void MCP::build(const AgentsLoader* _al, const FlatlandLoader* _ml, options _options1, int timestep)  // TODO: Ignore wait actions
{
    al = _al;
    ml = _ml;
    options1 = _options1;
    //if (options1.debug)
    //    cout << "Start MCP ..." << endl;
    size_t map_size = ml->cols * ml->rows;
    //if (options1.debug)
    //    cout << "map_size: " << map_size << endl;
    mcp.resize(map_size);
    agent_time.resize(al->getNumOfAllAgents(), 0);
    to_go.resize(al->getNumOfAllAgents(), -1);
    appear_time.resize(al->getNumOfAllAgents(), al->constraintTable.length_max + 1);
    size_t max_timestep = 0;
    for (int i = 0; i < al->getNumOfAllAgents(); i++)
        if (!al->paths_all[i].empty() && al->paths_all[i].size() > max_timestep)
            max_timestep = al->paths_all[i].size();
    //if (options1.debug)
    //    cout << "max_timestep = " << max_timestep << endl;

    // Push nodes to MCP
    no_wait_time.resize(al->getNumOfAllAgents());
    for (size_t t = 0; t < max_timestep; t++)
    {
        for (int i = 0; i < al->getNumOfAllAgents(); i++)
        {
            if (!al->paths_all[i].empty() && t < al->paths_all[i].size())
            {
                if (al->paths_all[i][t].location != -1)
                {
                    if (mcp[al->paths_all[i][t].location].empty())
                    {
                        mcp[al->paths_all[i][t].location].push_back(make_tuple(i, t));
                        no_wait_time[i].push_back(t);
                    }

                    else if (get<0>(mcp[al->paths_all[i][t].location].back()) != i)
                    {
                        mcp[al->paths_all[i][t].location].push_back(make_tuple(i, t));
                        no_wait_time[i].push_back(t);
                    }

                    else if (al->paths_all[i][t].location != al->paths_all[i][t-1].location)  // Detect cycle in paths
                    {
                        mcp[al->paths_all[i][t].location].push_back(make_tuple(i, t));
                        no_wait_time[i].push_back(t);
                    }

                    appear_time[i] = min(appear_time[i], (int)t + timestep);
                }
            }
        }
    }

    for (int i = 0; i < al->getNumOfAllAgents(); i++)
    {
        assert(al->paths_all[i].empty() || !no_wait_time[i].empty());
        if (!no_wait_time[i].empty() && no_wait_time[i][0] == 0)
        {
            agent_time[i] = 1;
            to_go[i] = al->paths_all[i][0].location;
            }
        }

    // Debug for no_wait_time
    // cout << endl;
    // al->printPaths();
    // for (int i = 0; i < al->getNumOfAllAgents(); i++)
    // {
    //     cout << "Size: " << al->paths_all[i].size() << endl;
    //     cout << "Size: " << no_wait_time[i].size() << endl;
    //     cout << "[ ";
    //     for (int t = 0; t < no_wait_time[i].size(); t++)
    //     {
    //         cout << no_wait_time[i][t] << ", ";
    //     }
    //     cout << " ]" << endl;
    // }
    // printAgentTime();
    //if (options1.debug)
    //    cout << "End building MCP ..." << endl;
}


void MCP::getNextLoc(p::list agent_location, int timestep)
{
    for (int i = 0; i < al->getNumOfAllAgents(); i++)
    {
        if (al->paths_all[i].empty() || al->agents_all[i].status >= 2)
            to_go[i] = -1;
        else if (al->agents_all[i].malfunction_left > 0)
            to_go[i] = p::extract<int>(p::long_(agent_location[i]));
        else if (!al->paths_all[i].empty() &&
            appear_time[i] <= timestep &&
            agent_time[i] < no_wait_time[i].size())
        {
            assert(!mcp[al->paths_all[i][no_wait_time[i][agent_time[i]]].location].empty());

            // if (i == 3 && al->paths_all[i][no_wait_time[i][agent_time[i] - 1]].location == 6563)
            // {
            //     cout << "********************************************" << endl;
            //     cout << "Agent: " << i << endl;
            //     cout << "agent_time: " << agent_time[i] << endl;
            //     cout << "no_wait_time: " << no_wait_time[i][agent_time[i]] << endl;
            //     cout << "Next location: " << al->paths_all[i][no_wait_time[i][agent_time[i]]].location << endl;
            //     cout << "MCP at next location: " << get<0>(mcp[al->paths_all[i][no_wait_time[i][agent_time[i]]].location].front()) << ", " <<
            //         get<1>(mcp[al->paths_all[i][no_wait_time[i][agent_time[i]]].location].front()) << endl;
            //     printMCP(6563);
            //     cout << "********************************************" << endl;
            //     sleep(10);
            // }

            int loc = al->paths_all[i][no_wait_time[i][agent_time[i]]].location;
            int first_agent = get<0>(mcp[loc].front());
            // int first_time = get<1>(mcp[loc].front());

            if (get<0>(mcp[loc].front()) == i || agent_time[i] == no_wait_time[i].size() - 1)
                to_go[i] = al->paths_all[i][no_wait_time[i][agent_time[i]]].location;

            else if (first_agent < i && mcp[loc].size() > 1)
            {
                if (get<0>(*std::next(mcp[loc].begin())) == i && // the second agent is i
                    agent_location[first_agent] == loc && // the fist agent is already at loc
                    to_go[first_agent] != loc &&
                    al->agents_all[first_agent].malfunction_left == 0)  // the first agent is going to leave
                    // agent_location[i] != al->paths_all[first_agent][agent_time[first_agent]].location) // not edge conflict
                {
                    to_go[i] = al->paths_all[i][no_wait_time[i][agent_time[i]]].location;
                }
                /*int next_agent = get<0>(*std::next(mcp[loc].begin()));
                int next_time = get<1>(*std::next(mcp[loc].begin()));  // equal to agent_time[next_agent]
                if (next_agent == i &&
                    appear_time[first_agent] < agent_time[first_agent] &&
                    al->paths_all[i][next_time-1].heading == al->paths_all[first_agent][first_time-1].heading)
                {
                    if (al->paths_all[next_agent][next_agent-1].location < 0)
                    {
                        if(agent_location[first_agent] == loc)
                        {
                            to_go[i] = al->paths_all[i][agent_time[i]].location;
                        }
                    }
                    else if (abs(ml->row_coordinate(al->paths_all[i][next_time-1].location) -
                        ml->row_coordinate(al->paths_all[first_agent][first_time-1].location)) +
                        abs(ml->col_coordinate(al->paths_all[i][next_time-1].location) -
                        ml->col_coordinate(al->paths_all[first_agent][first_time-1].location)) <= 1)
                    {
                        assert(agent_time[next_agent] == next_time);
                        to_go[i] = al->paths_all[i][agent_time[i]].location;
                    }
                    // else
                    // {
                    //     cout << "First agent: " << first_agent << endl;
                    //     cout << "First time: " << first_time << endl;
                    //     cout << "First time - 1 -> loc: " << ml->row_coordinate(al->paths_all[first_agent][first_time-1].location) << ", " << ml->col_coordinate(al->paths_all[first_agent][first_time-1].location) << endl;

                    //     cout << "Next agent: " << next_agent << endl;
                    //     cout << "Next time: " << next_time << endl;
                    //     cout << "Next time - 1 -> loc: " << ml->row_coordinate(al->paths_all[next_agent][next_time-1].location) << ", " << ml->col_coordinate(al->paths_all[next_agent][next_time-1].location) << endl;

                    //     cout << "Current Heading: " << al->paths_all[i][agent_time[i]-1].heading << endl;
                    //     cout << "Current location: " << ml->row_coordinate(al->paths_all[i][agent_time[i]].location) << ", " << ml->col_coordinate(al->paths_all[i][agent_time[i]].location) << endl;
                    //     cout << "Map size: " << ml->map_size() << endl;
                    //     cout << "Map size: " << ml->rows << ", " << ml->cols << endl;
                    //     cout << endl;
                    //     sleep(60);
                    //     assert(0);
                    // }
                }*/
            }
        }
    }
    /*if (options1.debug)
    {
        cout << "\t\t\t\tNext locations: ";
        for (const auto& loc : to_go)
            cout << loc << "\t";
        cout << endl;
    }*/
}


void MCP::update(p::list agent_location, p::dict agent_action)
{
    for (int i = 0; i < al->getNumOfAllAgents(); i++)
    {
        if (al->paths_all[i].empty())
            continue;
        // Reach Goal
        //else if (agent_time[i] == no_wait_time[i].size() &&
        //    !mcp[al->paths_all[i][no_wait_time[i][agent_time[i] - 1]].location].empty() &&
        //    get<0>(mcp[al->paths_all[i][no_wait_time[i][agent_time[i] - 1]].location].front()) == i)
        //{
        //    mcp[al->paths_all[i][no_wait_time[i][agent_time[i] - 1]].location].pop_front();

            // for (const auto& m: mcp)
            // {
            //     for (const  auto& a: m)
            //     {
            //         if (get<0>(a) == i)
            //         {
            //             cout << "Error in agent " << get<0>(a) << "at timestep " << get<1>(a) << endl;
            //             sleep(3);
            //         }
            //     }
            // }
        //}

        else if (agent_time[i] < no_wait_time[i].size() &&
                 agent_location[i] != -1 &&
                 agent_location[i] == al->paths_all[i][no_wait_time[i][agent_time[i]]].location)
        {
            /*if (options1.debug)
            {
                cout << "agent " << i << endl;
                cout << "agent_time[i]: " << no_wait_time[i][agent_time[i]] << endl;
                cout << "MCP: a->" << get<0>(mcp[al->paths_all[i][no_wait_time[i][agent_time[i]]].location].front());
                cout << "  t->" << get<1>(mcp[al->paths_all[i][no_wait_time[i][agent_time[i]]].location].front()) <<endl;
            }*/

            // Remove previous location from MCP after reach time no_wait_time[agent_time[i]]
            if (no_wait_time[i][agent_time[i]] > 0 && agent_time[i] > 0 &&
                al->paths_all[i][no_wait_time[i][agent_time[i] - 1]].location != -1 &&
                !mcp[al->paths_all[i][no_wait_time[i][agent_time[i] - 1]].location].empty())
            {
                assert(get<0>(mcp[al->paths_all[i][no_wait_time[i][agent_time[i] - 1]].location].front()) == i);
                // cout << "Pop mcp: " << al->paths_all[i][agent_time[i]-1].location << endl;
                mcp[al->paths_all[i][no_wait_time[i][agent_time[i] - 1]].location].pop_front();
            }

            if (agent_time[i] == (int) no_wait_time[i].size() - 1) // the agent reaches the goal location
            {
                auto p = mcp[al->paths_all[i].back().location].begin();
                while (get<0>(*p) != i)
                {
                    ++p;
                    assert(p != mcp[al->paths_all[i].back().location].end());
                }
                mcp[al->paths_all[i].back().location].erase(p);
            }
            agent_time[i] ++;
        }
        else if (al->agents_all[i].status == 1 &&
                al->paths_all[i][no_wait_time[i][agent_time[i] - 1]].location != agent_location[i])
        {
            cout << "Agent " << i << " at location " << p::extract<int>(p::long_(agent_location[i])) << " should be "
                << al->paths_all[i][no_wait_time[i][agent_time[i] - 1]].location << endl;
            cout << "BOOM" << endl;
        }
        else if (al->agents_all[i].status == 0)
        {
            assert(agent_time[i] == 0);
        }
    }

}


void MCP::printAll()
{
    cout << "==================== MCP ====================" << endl;
    for (int i = 0; i < mcp.size(); i++)
    {
        if (!mcp[i].empty())
        {
            cout << "[" << i << "]: ";
            auto &last = *(--mcp[i].end());
            for (const auto& p: mcp[i])
            {
                cout << "(" << get<0>(p) << "," << get<1>(p) << ")";
                if (&p != &last)
                    cout << "->";
                else
                    cout << endl;
            }
        }
    }
    cout << "\n================== MCP END ==================" << endl;
}


void MCP::print(int loc)
{
    cout << "==================== MCP ====================" << endl;
    if (loc < mcp.size() && !mcp[loc].empty())
    {
        cout << "[" << loc << "]: ";
        auto &last = *(--mcp[loc].end());
        for (const auto& p: mcp[loc])
        {
            cout << "(" << get<0>(p) << "," << get<1>(p) << ")";
            if (&p != &last)
                cout << "->";
            else
                cout << endl;
        }
    }
    cout << "\n================== MCP END ==================" << endl;
}


void MCP::printAgentTime()
{
    cout << "==================== Time ====================" << endl;
    for (int i = 0; i < al->getNumOfAllAgents(); i++)
    {
        cout << "Agent " << i << ": " << agent_time[i] << endl;
    }
    cout << "================== End Time ==================" << endl;
}


void MCP::printAgentNoWaitTime()
{
    cout << "==================== Time ====================" << endl;
    for (int i = 0; i < al->getNumOfAllAgents(); i++)
    {
        cout << "Agent " << i << ": ";
        for (int t = 0; t < no_wait_time[i].size(); t++)
            cout << no_wait_time[i][t] << ", ";
        cout << endl;
    }
    cout << "================== End Time ==================" << endl;
}


int MCP::getEstimatedCost(int timestep) const
{
    int cost = 0;
    for (int i = 0 ; i < (int)al->agents_all.size(); i++)
    {
        if (al->agents_all[i].status >= 2)  // done (2) or done removed (3)
            continue;
        else if (al->paths_all[i].empty())
            cost += al->constraintTable.length_max;
        else if (appear_time[i] > timestep)
            cost += (int)al->paths_all[i].size() - timestep;
        else if (agent_time[i] == 0)
            cost += (int)al->paths_all[i].size() - appear_time[i];
        else
            cost += (int)al->paths_all[i].size() - no_wait_time[i][agent_time[i] - 1];
    }
    return cost;
}

