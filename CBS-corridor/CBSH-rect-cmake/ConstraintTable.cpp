#include "ConstraintTable.h"
void ConstraintTable::insert(int loc, int t_min, int t_max)
{
    assert(loc >= 0);
    if (CT[loc].empty())
        CT[loc].resize(length_max + 1, false);
	if (t_max == 0) {
        //CT[loc].emplace(t_min);
        CT[loc][t_min] = true;
	}
	else {
	    for (int t = t_min; t < t_max; t++)
            //CT[loc].emplace(t);
            CT[loc][t] = true;
	}
}

bool ConstraintTable::is_constrained(int agent_id, int loc, int timestep) const
{
    if (loc < 0)
        return false;
    //if (CT[loc].count(timestep))
    if (!CT.empty() && !CT[loc].empty() && CT[loc][timestep])
        return true;

    if (CT_paths[loc].empty())
        return false;
    for (int t = timestep - kRobust; t <= timestep + kRobust; t++)
    {
        if (CT_paths[loc][t] >= 0)
        {
            assert(agent_id != CT_paths[loc][t]);
            if ((agent_id > CT_paths[loc][t] && timestep <= t) || // This agent reaches loc earlier than the second agent with smaller id
                (agent_id < CT_paths[loc][t] && timestep >= t))   // This agent reaches loc later than the second agent with larger id
                return true;
        }
    }
	return false;
}

void ConstraintTable::get_agents(set<int>& conflicting_agents, int loc) const
{
    if (loc < 0)
        return;

    for (auto agent : CT_paths[loc])
    {
        if (agent >= 0)
            conflicting_agents.insert(agent);
    }
}

void ConstraintTable::get_agents(list< pair<int, int> >& agents, int excluded_agent, const pair<int,int>& loc_time_pair) const
{
    int loc = loc_time_pair.first;
    for (int t = loc_time_pair.second; t < (int)CT_paths[loc].size(); t++)
    {
        int agent = CT_paths[loc][t];
        if (agent >= 0 && agent != excluded_agent && (agents.empty() || agents.back().first != agent))
        {
            agents.emplace_back(agent, t);
        }
    }
}

void ConstraintTable::get_agents(set<int>& conflicting_agents, int groupsize, int loc) const
{
    if (loc < 0 || CT_paths[loc].empty())
        return;
    int t_max = (int) CT_paths[loc].size() - 1;
    while (CT_paths[loc][t_max] < 0 && t_max > 0)
        t_max--;
    if (t_max == 0)
        return;
    int t0 = rand() % t_max;
    if (CT_paths[loc][t0] >= 0)
        conflicting_agents.insert(CT_paths[loc][t0]);
    int delta = 1;
    while (t0 - delta >= 0 || t0 + delta <= t_max)
    {
        if (t0 - delta >= 0 && CT_paths[loc][t0 - delta] >= 0)
        {
            conflicting_agents.insert(CT_paths[loc][t0 - delta]);
            if((int)conflicting_agents.size() == groupsize)
                return;
        }
        if (t0 + delta <= t_max && CT_paths[loc][t0 + delta] >= 0)
        {
            conflicting_agents.insert(CT_paths[loc][t0 + delta]);
            if((int)conflicting_agents.size() == groupsize)
                return;
        }
        delta++;
    }
}

void ConstraintTable::get_conflicting_agents(int agent_id, set<int>& conflicting_agents, int loc, int timestep) const
{
    if (loc < 0 || CT_paths[loc].empty())
        return;

    for (int t = timestep - kRobust; t <= timestep + kRobust; t++)
    {
        if (CT_paths[loc][t] >= 0)
        {
            if ((agent_id > CT_paths[loc][t] && timestep <= t) || // This agent reaches loc earlier than the second agent with smaller id
                (agent_id < CT_paths[loc][t] && timestep >= t))   // This agent reaches loc later than the second agent with larger id
                conflicting_agents.insert(CT_paths[loc][t]);
        }

    }
}

bool ConstraintTable::insert_path(int agent_id, const Path& path)
{
    for (int timestep = 0; timestep < (int)path.size(); timestep++)
    {
        int loc = path[timestep].location;
        if (loc == -1)
            continue;
        if (CT_paths[loc].empty())
            CT_paths[loc].resize(length_max + kRobust + 1, -1);
        for (int t = timestep; t <= timestep; t++)
        {
            if(CT_paths[loc][t] != -1 && CT_paths[loc][t] != agent_id) //TODO:: can be removed in the submission version
            {
                cout << "A conflict between " << agent_id << " and " << CT_paths[loc][t] <<
                        " at location " << loc << " at timestep "<< t << endl;
                return false;
            }
            CT_paths[loc][t] = agent_id;
        }
    }
    return true;
}

void ConstraintTable::delete_path(int agent_id, const Path& path)
{
    for (int timestep = 0; timestep < (int)path.size(); timestep++)
    {
        int loc = path[timestep].location;
        if (loc == -1)
            continue;
        assert(CT_paths[loc][timestep] == agent_id);
        CT_paths[loc][timestep] = -1;
    }
}

/*bool ConstraintTable::is_good_malfunction_location(int loc, int t)
{
    if (loc <0)
        return true;
	//if (CT_Single.count(loc)) {
		for (auto conT : CT_Single[loc]) {
			if (conT >= t) {
				return false;
			}
		}

	//}
	auto it = CT.find(loc);
	if (it == CT.end())
	{
		return true;
	}
	for (auto constraint : it->second)
	{
		if (constraint.first >= t)
			return false;
	}
	return true;
}*/

//void ConstraintTable::insert(int loc, int t_min, int t_max)
//{
//
//	for (int t = t_min; t < t_max; t++) {
//		CT[loc].insert(t);
//	}
//	if (loc == goal_location && t_max > length_min)
//	{
//		length_min = t_max;
//	}
//	if (t_max < INT_MAX && t_max > latest_timestep)
//	{
//		latest_timestep = t_max;
//	}
//}
//
//bool ConstraintTable::is_constrained(int loc, int t)
//{
//	auto it = CT.find(loc);
//	if (it == CT.end())
//	{
//		return false;
//	}
//	
//	if (CT[loc].count(t))
//		return true;
//	else
//		return false;
//}