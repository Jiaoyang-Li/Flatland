/*
 * @author: Team An_old_driver
 * @created: 09-2020
 * Copyright (c) 2020 The University of Southern California. All Rights Reserved.
 * Copyrights licensed under an Academic/non-profit use license.
 * See the accompanying LICENSE file for terms.
*/

#include "MCP.h"


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
            assert(al->paths_all[i][no_wait_time[i][copy_agent_time[i] - 1]].location ==
                al->agents_all[i].position);
            paths[i].push_back(al->paths_all[i][no_wait_time[i][copy_agent_time[i] - 1]]);
        }
        else // the agent has not started to move yet
        {
            assert(al->agents_all[i].status == 0);
            paths[i].push_back(al->paths_all[i].front());
        }
        paths[i].back().position_fraction = al->agents_all[i].position_fraction;
        paths[i].back().malfunction_left = al->agents_all[i].malfunction_left;
        paths[i].back().exit_heading = al->agents_all[i].exit_heading;
        if (al->agents_all[i].position_fraction >= 1)
        {
            if (copy_agent_time[i] < (int) no_wait_time[i].size())
                paths[i].back().exit_loc = al->paths_all[i][no_wait_time[i][copy_agent_time[i]]].location;
            else
                paths[i].back().exit_loc = paths[i].back().location;
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
    //for (int i : unfinished_agents)
    //{
    //    if (al->agents_all[i].status == 0 && // the agent has not started yet
    //        !paths[i].empty() &&
    //        paths[i].back().location != ml->linearize_coordinate(al->agents_all[i].goal_location)) // it cannot reach its goal location
    //        paths[i].clear();
    //}
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
    if (copy_agent_time[i] == (int) no_wait_time[i].size()) // the agent has reached the last location on its path
    {
        int loc = al->agents_all[i].goal_location;
        if (paths[i][t].location == loc)// the agent has reached its goal location
        {
            assert(copy_mcp[loc].front() == i);
            copy_mcp[loc].pop_front();
            p = unfinished_agents.erase(p);
            return true;
        }
        else // the path is finished, but the agent still has to wait here
        {
            paths[i].push_back(paths[i].back()); // stay still
            return false;
        }
    }
    if (al->agents_all[i].malfunction_left > t + 1 ||
        (al->agents_all[i].malfunction_left > t && copy_agent_time[i] > 0)) // the agent is still in mal
    { // mal function does not affect the action of appearing at the start location
        // therefore, the agent can appear at the start location when malfunction_left - t = 1
        paths[i].push_back(paths[i].back()); // stay still
        paths[i].back().malfunction_left = max(0, paths[i].back().malfunction_left - 1);
        ++p;
        return false;
    }
    if (appear_time[i] > t + 1 + timestep) // the agent should not appear before the appear time
    {
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


void MCP::getNextLoc(int timestep)
{
    vector<bool> updated(al->getNumOfAllAgents(), false);
    to_go.resize(al->getNumOfAllAgents(), -1);
    if (options1.debug)
        cout << "Timestep=" << timestep << ": ";
    for (int i : active_agents)
    {
        getNextLocForAgent(i, updated, timestep);
        if (options1.debug)
        {
            cout << i << "[" << al->agents_all[i].position << "->" << to_go[i];
            if (al->agents_all[i].position_fraction > 0)
            {
                list<Transition> temp;
                ml->get_transitions(temp, al->agents_all[i].position, al->agents_all[i].heading, true);
                if (temp.size() == 1)
                    cout << "(" << temp.front().location << ")";
                else
                    cout << "(" << al->agents_all[i].position + ml->moves_offset[al->agents_all[i].exit_heading] << ")";

            }

            cout << "],";
        }
    }
    if (options1.debug)
        cout << endl;
}

void MCP::getNextLocForAgent(int i, vector<bool>& updated, int timestep)
{
    if (updated[i])
        return;
    updated[i] = true;
    if (al->agents_all[i].status >= 2 || // the agent is done
        al->paths_all[i].empty() ||  // the agent does not have paths
        appear_time[i] > timestep)  // the agent cannot show up now
    {
        to_go[i] = -1; // do noting
        return;
    }
    assert(al->agents_all[i].position < 0 || mcp[al->agents_all[i].position].front() == i);
    if (al->agents_all[i].malfunction_left > 0) // the agent is malfunctioning
    {
        to_go[i] = al->agents_all[i].position; // stand still
        return;
    }
    if (agent_time[i] == no_wait_time[i].size()) // the agent has reached the last location on its path, which is not its goal location
    {
        assert(al->agents_all[i].position > 0 && al->agents_all[i].position != al->agents_all[i].goal_location);
        to_go[i] = al->agents_all[i].position; // stand still
        return;
    }

    assert(!mcp[al->paths_all[i][no_wait_time[i][agent_time[i]]].location].empty());
    int loc = al->paths_all[i][no_wait_time[i][agent_time[i]]].location;
    int first_agent = mcp[loc].front();

    if (mcp[loc].front() == i || // the agent is the first in the mcp
        loc == al->agents_all[i].goal_location) // the agent is going to reach its goal location
    {
        to_go[i] = loc;
        return;
    }

    assert(mcp[loc].size() > 1); // the agent is not the first in the mcp, so mcp should have at least two agents

    if (*std::next(mcp[loc].begin()) == i && // the second agent is i
        al->agents_all[first_agent].position == loc && // the fist agent is already at loc
        al->agents_all[first_agent].malfunction_left == 0)  // the first agent does not malfunction
    {
        if (!updated[first_agent])
        {
            to_go[i] = loc; // pretend that this agent can move
            getNextLocForAgent(first_agent, updated, timestep); // update the first agent
        }
        if (to_go[first_agent] == loc) // the first agent does not move
        { // this agent cannot move either
            to_go[i] = al->agents_all[i].position; // stand still
        }
        else
            to_go[i] = loc; // this agent can move
    }
    else
        to_go[i] = al->agents_all[i].position; // stand still
}

void MCP::update()
{
    for (auto pt = active_agents.begin(); pt !=  active_agents.end();)
    {
        int i = *pt;
        if (agent_time[i] < no_wait_time[i].size() &&
            al->agents_all[i].position == al->paths_all[i][no_wait_time[i][agent_time[i]]].location)
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

            if (al->agents_all[i].position == al->agents_all[i].goal_location) // the agent reaches the goal location
            {
                assert(agent_time[i] == (int) no_wait_time[i].size() - 1);
                auto p = mcp[al->agents_all[i].goal_location].begin();
                while (*p != i)
                {
                    ++p;
                    assert(p != mcp[al->agents_all[i].goal_location].end());
                }
                mcp[al->agents_all[i].goal_location].erase(p);
            }
            agent_time[i] ++;
        }
        else
        {
            assert(al->agents_all[i].status != 1 ||
                   al->paths_all[i][no_wait_time[i][agent_time[i] - 1]].location == al->agents_all[i].position);
            assert(al->agents_all[i].status != 0 || agent_time[i] == 0);
        }
        if (al->agents_all[i].status >= 2)
        {
            pt = active_agents.erase(pt);
        }
        else
        {
            ++pt;
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

