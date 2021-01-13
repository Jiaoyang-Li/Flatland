/*
 * @author: Team An_old_driver
 * @created: 09-2020
 * Copyright (c) 2020 The University of Southern California. All Rights Reserved.
 * Copyrights licensed under an Academic/non-profit use license.
 * See the accompanying LICENSE file for terms.
*/

#include "CPR.h"


CPR::CPR(AgentsLoader& al, FlatlandLoader& ml, const options& options1, float time_limit):
    al(al), ml(ml), options1(options1)
{
    start_time = Time::now();
    agent_steps.resize(al.getNumOfAllAgents(), 0);
    to_go.resize(al.getNumOfAllAgents(), -1);
    // group the agents by their start location and initial heading
    unordered_map< int, list<Agent*> > agent_groups;
    for (auto & agent : al.agents_all)
    {
        auto key = ml.map_size() * agent.heading + agent.initial_location;
        agent_groups[key].push_back(&agent);
    }

    // sort agent_groups according to their cardinality
    list<list<Agent*> > sorted_groups;
    for (const auto & group : agent_groups)
    {
        auto it = sorted_groups.begin();
        for (; it != sorted_groups.end(); ++it)
        {
            if (it->size() < group.second.size())
            {
                sorted_groups.insert(it, group.second);
                break;
            }
        }
        if (it == sorted_groups.end())
        {
            sorted_groups.insert(it, group.second);
        }
    }

    // put all agents into the unplanned_agents queue in the sorted group order
    for (const auto & group : sorted_groups)
    {
        for (const auto & agent : group)
        {
            unplanned_agents.push_back(agent);
        }
    }
    runtime = ((fsec) (Time::now() - start_time)).count();
    replan = true;
    planPaths(time_limit - runtime);// plan paths for the agents

}

void CPR::getNextLoc()
{
    for (int i = 0; i < al.getNumOfAllAgents(); i++)
    {
        if (agent_steps[i]+1 < al.paths_all[i].size() )
        {
            to_go[i] = al.paths_all[i][agent_steps[i] + 1].location;
            //cout << "\t\t\t Agent " << i << " wants to move from " << al.paths_all[i][agent_steps[i]].location
            //    << " to " <<  to_go[i] << endl;
        }
        else
            to_go[i] = -1;
    }
}


void CPR::update()
{
    replan = false;
    for (int i = 0; i < al.getNumOfAllAgents(); i++)
    {
        if (agent_steps[i]+1 >= al.paths_all[i].size()) // the agent either has finished or the path has not been planned yet
            continue;
        assert(al.agents_all[i].position == al.paths_all[i][agent_steps[i]].location ||
                       al.agents_all[i].position == al.paths_all[i][agent_steps[i] + 1].location);
        if (al.agents_all[i].position == al.paths_all[i][agent_steps[i] + 1].location) // the agent has successfully moved to the next location
        {
            //cout << "Agent " << i << " moves from " << al.paths_all[i][agent_steps[i]].location << " to " << al.agents_all[i].position << endl;
            if (al.paths_all[i][agent_steps[i]].location >= 0)
            {
                // update highway's value in both directions
                int from = al.paths_all[i][agent_steps[i]].location;
                int to = al.paths_all[i][agent_steps[i] + 1].location;
                int heading = ml.getHeading(from, to);
                ml.railMap[from].highways[heading]--;
                assert(ml.railMap[from].highways[heading] >= 0);
                if (ml.railMap[from].highways[heading] == 0 && !unplanned_agents.empty())
                {
                    replan = true; // a new edge is released. Let's try to replan the unplanned agents later
                }
            }
            // agent moves to the next state on its path
            agent_steps[i]++;
            assert(al.agents_all[i].position != al.paths_all[i][agent_steps[i] + 1].location);
        }
    }
    //if (!unplanned_agents.empty())
    //{
    //    cout << unplanned_agents.size() << " agents unplanned" << endl;
    //    int count = 0;
    //    for (int i = 0; i < ml.cols * ml.rows; i++)
     //       for (int j = 0; j < 4; j++)
    //            count += ml.railMap[i].highways[j];
    //    cout << count << " Reservations" << endl;
    //}
}


void CPR::planPaths(float time_limit)
{
    if (!replan)
        return;
    start_time = Time::now();
    for (auto it = unplanned_agents.begin(); it != unplanned_agents.end();)
    {
        auto agent = *it;
        runtime = ((fsec) (Time::now() - start_time)).count();
        if (runtime > time_limit)
            return;
        al.agents.resize(1);
        al.agents[0] = agent;
        SinglePlanning planner(ml, al, 1, time_limit - runtime, options1);
        planner.search(true);
        if (!planner.path.empty())
        {
            al.paths_all[agent->agent_id] = planner.path;
            for (int t = 1; t < (int)planner.path.size(); t++)
            {
                int from = planner.path[t - 1].location;
                int to = planner.path[t].location;
                assert(from != to && to >= 0);
                if (from >= 0)
                {
                    int heading = ml.getHeading(from, to);
                    assert(heading == planner.path[t].heading);
                    // update highway's value in both directions
                    ml.railMap[from].highways[heading]++;
                }
            }
            it = unplanned_agents.erase(it);
            //cout << "Find a path for agent " << agent->agent_id << endl;
        }
        else
        {
            ++it;
        }
    }
    //int count = 0;
    //for (int i = 0; i < ml.cols * ml.rows; i++)
    //    for (int j = 0; j < 4; j++)
    //        count += ml.railMap[i].highways[j];
    //cout << count << " Reservations" << endl;
}