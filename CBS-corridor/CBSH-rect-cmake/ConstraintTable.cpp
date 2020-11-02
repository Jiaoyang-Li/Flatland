#include "ConstraintTable.h"


bool ConstraintTable::is_constrained(int agent_id, int loc, int timestep, int pre_loc) const
{
    if (loc < 0 || CT_paths[loc].empty())
        return false;
    if (timestep > length_max)
        return true;
    assert(agent_id != CT_paths[loc][timestep]);
    return CT_paths[loc][timestep] >= 0 || // vertex conflict
           (pre_loc >= 0 && !CT_paths[pre_loc].empty() && timestep - 1 >= 0 && CT_paths[loc][timestep - 1] >= 0 &&
            CT_paths[loc][timestep - 1] == CT_paths[pre_loc][timestep]); //edge conflict
}

bool ConstraintTable::blocked(int loc, int t) const
{
    return t > length_max || (loc >=0 && !CT_paths[loc].empty() && CT_paths[loc][t] >= 0);
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

void ConstraintTable::get_agents(list< pair<int, int> >& agents, int excluded_agent,
        const tuple<int, int, int>& loc_timeinterval_pair) const
{
    int loc = std::get<0>(loc_timeinterval_pair);
    int t_max = std::min(std::get<2>(loc_timeinterval_pair), (int)CT_paths[loc].size());
    for (int t = std::get<1>(loc_timeinterval_pair); t < t_max; t++)
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

    if (CT_paths[loc][timestep] >= 0)
    {
        conflicting_agents.insert(CT_paths[loc][timestep]);
    }
}

bool ConstraintTable::insert_path(int agent_id, const Path& path)
{
    for (int timestep = (int)path.size() - 1; timestep >= 0; timestep--)
    {
        int loc = path[timestep].location;
        if (loc == -1)
            return true;
        if (CT_paths[loc].empty())
            CT_paths[loc].resize(length_max + 1, -1);

        assert(CT_paths[loc][timestep] == -1); // should not have vertex conflict
        //if(CT_paths[loc][timestep] != -1 && CT_paths[loc][timestep] != agent_id) //TODO:: can be removed in the submission version
        //{
        //    cout << "A conflict between " << agent_id << " and " << CT_paths[loc][timestep] <<
        //            " at location " << loc << " at timestep "<< timestep << endl;
        //    assert(false && "Find conflict");
        //    return false;
        //}
        assert(timestep == 0 || path[timestep-1].location == -1 || CT_paths[path[timestep-1].location].empty() ||
            CT_paths[path[timestep-1].location][timestep] != CT_paths[loc][timestep-1] ||
            CT_paths[loc][timestep-1] == -1);  // should not have edge conflict
        //if(timestep>=1 && path[timestep-1].location!= -1 && !CT_paths[path[timestep-1].location].empty() &&
        // CT_paths[path[timestep-1].location][timestep] == CT_paths[loc][timestep-1] && CT_paths[loc][timestep-1]!=-1) //TODO:: can be removed in the submission version
        //{
        //    cout << "A edge conflict between " << agent_id << " and " << CT_paths[loc][timestep-1] <<
        //         " at location " << loc<<","<<path[timestep-1].location << " at timestep "<< timestep << endl;
        //    assert(false &&"Find edge conflict");
        //    return false;
        //}
        CT_paths[loc][timestep] = agent_id;
        latest_conatraints[loc] = std::max(latest_conatraints[loc], timestep);
    }
    return true;
}

bool ConstraintTable::insert_path_list(int agent_id, const Path& path)
{
    for (int timestep = (int)path.size() - 1; timestep >= 0; timestep--)
    {
        int loc = path[timestep].location;
        int to;
        if (timestep <(int)path.size() - 2)
            to = path[timestep+1].location;
        else
            to = -1;
        if (loc == -1)
            return true;
        if (CT_paths[loc].empty())
            CT_paths_list[loc].emplace_back(timestep,agent_id,to);
        else if(timestep < CT_paths_list[loc].front().timestep){
            CT_paths_list[loc].emplace_front(timestep,agent_id,to);
        }
        else if(timestep > CT_paths_list[loc].back().timestep){
            CT_paths_list[loc].emplace_back(timestep,agent_id,to);
        }
        else if(CT_paths[loc].size() == 1 && timestep == CT_paths_list[loc].front().timestep){
            assert(false && "Vertex conflict");
        }
        else{
            auto it = CT_paths_list[loc].begin();
            auto pre = it;
            it++;
            while (it!=CT_paths_list[loc].end()){
                assert(it->timestep!=timestep && "Vertex conflict");
                if (it->timestep!=timestep)
                    break;

                if (timestep > pre->timestep && timestep < it->timestep) {
                    CT_paths_list[loc].insert(pre, CT_entry(timestep, agent_id,to));
                    break;
                }
                pre = it;
                it++;
            }

        }
    }
    return true;
}

void ConstraintTable::delete_path(int agent_id, const Path& path)
{
    for (int timestep = (int)path.size() - 1; timestep >= 0; timestep--)
    {
        int loc = path[timestep].location;
        if (loc == -1)
            break;
        assert(CT_paths[loc][timestep] == agent_id);
        CT_paths[loc][timestep] = -1;
        if (latest_conatraints[loc] == timestep)
        {
            int t = timestep - 1;
            while(t >= 0 && CT_paths[loc][t] < 0)
                t--;
            latest_conatraints[loc] = t;
        }
    }
}

void ConstraintTable::delete_path_list(int agent_id, const Path& path)
{
    for (int timestep = (int)path.size() - 1; timestep >= 0; timestep--)
    {
        int loc = path[timestep].location;
        if (loc == -1)
            break;
        assert(!CT_paths_list[loc].empty());

        if(timestep == CT_paths_list[loc].front().timestep){
            assert(CT_paths_list[loc].front().agent_id == agent_id);
            CT_paths_list[loc].pop_front();
        }
        else if(timestep == CT_paths_list[loc].back().timestep){
            assert(CT_paths_list[loc].back().agent_id == agent_id);
            CT_paths_list[loc].pop_back();        }
        else{
            auto it = CT_paths_list[loc].begin();
            while (it!=CT_paths_list[loc].end()){
                assert(it->timestep!=timestep && "Vertex conflict");
                if (it->timestep!=timestep) {
                    CT_paths_list[loc].erase(it);
                    break;
                }
                else
                    ++it;
            }

        }

    }
}

bool ConstraintTable::is_constrained_list(int agent_id, int loc, int timestep, int pre_loc) const
{
    if (loc < 0 || CT_paths_list[loc].empty())
        return false;

    if (timestep <= CT_paths_list[loc].front().timestep){
        return timestep == CT_paths_list[loc].front().timestep;
    }
    else if (timestep >= CT_paths_list[loc].back().timestep){
        return timestep == CT_paths_list[loc].back().timestep;
    }
    else{

        auto it = CT_paths_list[loc].begin();
        while (it!=CT_paths_list[loc].end()){
            if (it->timestep > timestep)
                break;

            if (timestep == it->timestep || (it->to!= -1 && timestep - it->timestep == 1 && pre_loc == it->to ))
                return true;


        };
    }

}