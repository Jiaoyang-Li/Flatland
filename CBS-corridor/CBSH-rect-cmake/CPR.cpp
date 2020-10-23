#include "CPR.h"


CPR::CPR(AgentsLoader& al, FlatlandLoader& ml, const options& options1, float time_limit):
    al(al), ml(ml), options1(options1)
{
    start_time = Time::now();
    agent_steps.resize(al.getNumOfAllAgents(), 0);
    // group the agents by their start location and initial heading
    unordered_map< int, list<Agent*> > agent_groups;
    for (auto it = al.agents_all.begin(); it != al.agents_all.end(); ++it)
    {
        auto key = ml.map_size() * it->heading + ml.linearize_coordinate(it->initial_location);
        agent_groups[key].push_back(&(*it));
    }

    // sort agent_groups according to their cardinality
    list<list<Agent*> > sorted_groups;
    for (const auto& group : agent_groups)
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
    for (const auto& group : sorted_groups)
    {
        for (const auto &agent : group)
        {
            unplanned_agents.push_back(agent);
        }
    }
    runtime = ((fsec) (Time::now() - start_time)).count();
    planPaths(time_limit - runtime);// plan paths for the agents

}

void CPR::getNextLoc(vector<int>& to_go)
{
    for (int i = 0; i < al.getNumOfAllAgents(); i++)
    {
        if (al.paths_all[i].empty() || // the path has not been planned yet
            al.agents_all[i].status >= 2) // the agent has already finished
            continue;
        assert(agent_steps[i]+1 <= al.paths_all[i].size() );
        to_go[i] = al.paths_all[i][agent_steps[i]].exit_loc;
    }
}


void CPR::update(p::list agent_location)
{
    replan = false;
    for (int i = 0; i < al.getNumOfAllAgents(); i++)
    {
        if (agent_steps[i]+1 >= al.paths_all[i].size()) // the agent either has finished or the path has not been planned yet
            continue;

        if (agent_location[i] == al.paths_all[i][agent_steps[i]].exit_loc) // the agent has successfully moved to the next location
        {
            // update highway's value in both directions
            auto state = al.paths_all[i][agent_steps[i]];
            ml.railMap[state.location].highways[state.exit_heading]--;
            ml.railMap[state.exit_loc].highways[(state.exit_heading + 2) % 4]++;
            // agent moves to the next state on its path
            agent_steps[i]++;
            if (ml.railMap[state.location].highways[state.exit_heading] == 0 && !unplanned_agents.empty())
            {
                replan = true; // a new edge is released. Let's try to replan the unplanned agents later
            }
        }
    }

}


void CPR::planPaths(float time_limit)
{
    start_time = Time::now();
    for (auto it = unplanned_agents.begin(); it != unplanned_agents.end(); ++it)
    {
        auto agent = *it;
        runtime = ((fsec) (Time::now() - start_time)).count();
        if (runtime > time_limit)
            return;
        al.agents.resize(1);
        al.agents[0] = agent;
        SinglePlanningFlat planner(ml, al, 1, time_limit - runtime, options1);
        planner.search();
        if (!planner.path.empty())
        {
            al.paths_all[agent->agent_id] = planner.path;
            for (const auto& state : planner.path)
            {
                if (state.location != state.exit_loc && state.location >= 0 && state.exit_heading >= 0)
                {
                    // update highway's value in both directions
                    ml.railMap[state.location].highways[state.exit_heading]++;
                    ml.railMap[state.exit_loc].highways[(state.exit_heading + 2) % 4]--;
                }
            }
            unplanned_agents.erase(it);
        }
    }
}