#include "MCP.h"

// TODO: if the agent is already appeared on the map,
//  but because of the delays, its replanned path does not reach its goal location,
//  so we cannot assume that the last location in an agent's path is its goal location
void MCP::simulate(vector<Path>& paths, int timestep)
{
    copy_agent_time = agent_time;
    copy_mcp = mcp;
    paths.resize(al->paths_all.size());
    unfinished_agents.clear();
    for (int i = 0; i < al->getNumOfAllAgents(); i++)
    {
        if (al->agents_all[i].status >= 2 || // the agent has already reached its goal location
            al->paths_all[i].empty()) // the agent does not have a path
            continue;
        unfinished_agents.push_back(i);
        paths[i].reserve(al->paths_all[i].size() * 2);
        if (copy_agent_time[i] > 0)
        {
            assert(copy_agent_time[i] < (int)no_wait_time[i].size());
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
        auto old_size = unfinished_agents.size();
        for (auto p = unfinished_agents.begin(); p != unfinished_agents.end();) {
            //if (timestep == 4 && t == 112)
            //    cout << "Timestep 112" << endl;
            moveAgent(paths, p, t, timestep);
        }
        if (options1.debug)
        {
            vector<int> occupied(ml->map_size(), -1);
            for (int i : unfinished_agents)
            {
                if (paths[i][t+1].location < 0)
                    continue;
                if (occupied[paths[i][t+1].location] >= 0)
                {
                    auto a1 = i;
                    auto a2 = occupied[paths[i][t+1].location];
                    cout << "MCP at location " << paths[i][t+1].location << ": ";
                    for (auto x : mcp[paths[i][t+1].location])
                        cout << x << "->";
                    cout << "Copied MCP at location " << paths[i][t+1].location << ": ";
                    for (auto x : copy_mcp[paths[i][t+1].location])
                        cout << x << "->";
                    cout << endl;
                    cout << "Agent " << a1 << " moves from " << paths[a1][t].location << " to " << paths[a1][t+1].location << endl;
                    cout << "Agent " << a2 << " moves from " << paths[a2][t].location << " to " << paths[a2][t+1].location << endl;
                }
                assert(occupied[paths[i][t+1].location] < 0);
                occupied[paths[i][t+1].location] = i;
            }
        }
        if (options1.debug && !unfinished_agents.empty() && old_size == unfinished_agents.size())
        {
            bool move = false;
            for (int i : unfinished_agents)
            {
                if (paths[i][t+1].location != paths[i][t].location)
                    move = true;
                else if (al->agents_all[i].malfunction_left > t)
                    move = true;
            }
            if (!move)
            {
                cout << "Timestep=" << timestep << " and " << "t=" << t << endl;
                for (int i : unfinished_agents)
                {
                    assert(paths[i][t+1].location == paths[i][t].location);
                    int loc = al->paths_all[i][no_wait_time[i][copy_agent_time[i]]].location;
                    cout << "Agent " << i << ": " << paths[i][t+1].location << "->" << loc <<
                        ", which has to be first visited by " << copy_mcp[loc].front() << endl;
                }
            }
            assert(move);
        }
    }
    for (int i : unfinished_agents)
    {
        if (!paths[i].empty() && paths[i].back().location != ml->linearize_coordinate(al->agents_all[i].goal_location))
            paths[i].empty();
    }
}

bool MCP::moveAgent(vector<Path>& paths, list<int>::iterator& p, int t, int timestep)
{
    int i = *p;
    if (paths[i].size() == t + 2)  // we have already made the movement decision for the agent
    {
        ++p;
        return false;
    }
    assert(paths[i].size() == t + 1);
    assert(paths[i][t].location < 0 || copy_mcp[paths[i][t].location].front() == i);
    assert(copy_agent_time[i] <= (int) no_wait_time[i].size());
    if (copy_agent_time[i] == (int) no_wait_time[i].size())  // the agent has reached its location
    {
        int previous = al->paths_all[i][no_wait_time[i][copy_agent_time[i] - 1]].location;
        assert(copy_mcp[previous].front() == i);
        copy_mcp[previous].pop_front();
        p = unfinished_agents.erase(p);
        return true;
    }
    if (appear_time[i] > t + 1 + timestep) // the agent should not appear before the appear time
    {
        paths[i].push_back(paths[i].back()); // stay still
        ++p;
        return false;
    }
    if (al->agents_all[i].malfunction_left > t + 1 ||
             (al->agents_all[i].malfunction_left > t && copy_agent_time[i] > 0)) // the agent is still in mal
    { // mal function does not affect the action of appearing at the start location
        // therefore, the agent can appear at the start location when malfunction_left - t = 1
        paths[i].push_back(paths[i].back()); // stay still
        ++p;
        return false;
    }
    // check mcp to determine whether the agent should move or wait
    int loc = al->paths_all[i][no_wait_time[i][copy_agent_time[i]]].location;
    assert(!copy_mcp[loc].empty());
    if (copy_mcp[loc].front() == i)
    {
        paths[i].push_back(al->paths_all[i][no_wait_time[i][copy_agent_time[i]]]); // move
        if (copy_agent_time[i] > 0)
        {
            int previous = al->paths_all[i][no_wait_time[i][copy_agent_time[i] - 1]].location;
            if (copy_mcp[previous].front() != i)
            {
                for (auto item : copy_mcp[previous])
                    cout << item << ", ";
                cout << endl;
            }
            assert(copy_mcp[previous].front() == i);
            copy_mcp[previous].pop_front();
        }
        copy_agent_time[i]++;
        ++p;
        return true;
    }
    assert(copy_mcp[loc].size() > 1);
    int first_agent = copy_mcp[loc].front();
    if (*std::next(copy_mcp[loc].begin()) == i &&  // the second agent is i
        paths[first_agent].size() == t + 1 &&  // we have not made the movement decision for the first agent
        paths[first_agent][t].location == loc &&  // the fist agent is already at loc
        al->agents_all[first_agent].malfunction_left <= t)  // the first agent is able to leave
    {
        // pretend this agent can move: see whether the first agent can move successfully
        paths[i].push_back(al->paths_all[i][no_wait_time[i][copy_agent_time[i]]]); // move
        bool mcp_pop = false;
        if (copy_agent_time[i] > 0)
        {
            mcp_pop = true;
            int previous = al->paths_all[i][no_wait_time[i][copy_agent_time[i] - 1]].location;
            assert(copy_mcp[previous].front() == i);
            copy_mcp[previous].pop_front();
        }
        copy_agent_time[i]++;

        auto p2 = std::find(unfinished_agents.begin(), unfinished_agents.end(), first_agent);
        assert(p2 != unfinished_agents.end());
        auto succ = moveAgent(paths, p2, t, timestep);
        if (succ)
        {
            ++p;
            return true;
        }
        else // this agent cannot move
        {
            paths[i][t + 1] = paths[i][t];
            copy_agent_time[i]--;
            if (mcp_pop)
            {
                int previous = al->paths_all[i][no_wait_time[i][copy_agent_time[i] - 1]].location;
                copy_mcp[previous].push_front(i);
            }
            ++p;
            return false;
        }
    }
    paths[i].push_back(paths[i].back()); // stay still
    ++p; // next agent
    return false;
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
    active_agents.clear();
    for (int i = 0; i < al->getNumOfAllAgents(); i++)
        if (!al->paths_all[i].empty())
        {
            active_agents.push_back(i);
            if (al->paths_all[i].size() > max_timestep)
                max_timestep = al->paths_all[i].size();
        }
    //if (options1.debug)
    //    cout << "max_timestep = " << max_timestep << endl;

    // Push nodes to MCP
    no_wait_time.resize(al->getNumOfAllAgents());
    for (size_t t = 0; t < max_timestep; t++)
    {
        for (int i : active_agents)
        {
            if (t < al->paths_all[i].size() &&
                al->paths_all[i][t].location != -1 &&
                (t==0 || al->paths_all[i][t].location != al->paths_all[i][t-1].location))
            {
                mcp[al->paths_all[i][t].location].push_back(i);
                no_wait_time[i].push_back(t);
                appear_time[i] = min(appear_time[i], (int)t + timestep);
            }
        }
    }

    for (int i : active_agents)
    {
        assert(!no_wait_time[i].empty());
        if (!no_wait_time[i].empty() && no_wait_time[i][0] == 0)
        {
            agent_time[i] = 1;
            to_go[i] = al->paths_all[i][0].location;
        }
    }
}


void MCP::getNextLoc(p::list agent_location, int timestep)
{
    for (int i : active_agents)
    {
        if (al->agents_all[i].status >= 2)
            to_go[i] = -1;
        else if (al->agents_all[i].malfunction_left > 0)
            to_go[i] = p::extract<int>(p::long_(agent_location[i]));
        else if (!al->paths_all[i].empty() &&
            appear_time[i] <= timestep &&
            agent_time[i] < no_wait_time[i].size())
        {
            assert(!mcp[al->paths_all[i][no_wait_time[i][agent_time[i]]].location].empty());

            int loc = al->paths_all[i][no_wait_time[i][agent_time[i]]].location;
            int first_agent = mcp[loc].front();

            if (mcp[loc].front() == i || agent_time[i] == no_wait_time[i].size() - 1)
                to_go[i] = al->paths_all[i][no_wait_time[i][agent_time[i]]].location;

            else if ( mcp[loc].size() > 1)
            {
                if (*std::next(mcp[loc].begin()) == i && // the second agent is i
                    agent_location[first_agent] == loc && // the fist agent is already at loc
//                    to_go[first_agent] != loc &&
                    al->agents_all[first_agent].malfunction_left == 0)  // the first agent is going to leave
                    // agent_location[i] != al->paths_all[first_agent][agent_time[first_agent]].location) // not edge conflict
                {
                    to_go[i] = al->paths_all[i][no_wait_time[i][agent_time[i]]].location;

                }
                else if(al->agents_all[i].status==0){
                    to_go[i] = -1;
                }




            }

        }
    }
}


void MCP::update(p::list agent_location, p::dict agent_action)
{

    for (int i : active_agents)
    {
        if (agent_time[i] < no_wait_time[i].size() &&
                 agent_location[i] != -1 &&
                 agent_location[i] == al->paths_all[i][no_wait_time[i][agent_time[i]]].location)
        {
            // Remove previous location from MCP after reach time no_wait_time[agent_time[i]]
            if (no_wait_time[i][agent_time[i]] > 0 && agent_time[i] > 0 &&
                al->paths_all[i][no_wait_time[i][agent_time[i] - 1]].location != -1 &&
                !mcp[al->paths_all[i][no_wait_time[i][agent_time[i] - 1]].location].empty())
            {
                assert(mcp[al->paths_all[i][no_wait_time[i][agent_time[i] - 1]].location].front() == i);
                // cout << "Pop mcp: " << al->paths_all[i][agent_time[i]-1].location << endl;
                mcp[al->paths_all[i][no_wait_time[i][agent_time[i] - 1]].location].pop_front();
            }

            if (agent_time[i] == (int) no_wait_time[i].size() - 1) // the agent reaches the goal location
            {
                auto p = mcp[al->paths_all[i].back().location].begin();
                while (*p != i)
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
                cout << p;
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
            cout << p;
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

