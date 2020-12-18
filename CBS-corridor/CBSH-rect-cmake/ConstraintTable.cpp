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
        if (agent >= 0 && agent < num_of_agents)
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
        if (agent >= 0 && agent < num_of_agents && agent != excluded_agent && (agents.empty() || agents.back().first != agent))
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
    if (CT_paths[loc][t0] >= 0 && CT_paths[loc][t0] < num_of_agents)
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

    if (CT_paths[loc][timestep] >= 0 && CT_paths[loc][timestep] < num_of_agents)
    {
        conflicting_agents.insert(CT_paths[loc][timestep]);
    }
}

bool ConstraintTable::insert_path(int agent_id, const Path& path, vector<Path>* paths_all, float malfunction_rate)
{
    for (int timestep = (int)path.size() - 1; timestep >= 0; timestep--)
    {
        int loc = path[timestep].location;
        if (loc == -1)
            break;
        if (CT_paths[loc].empty())
            CT_paths[loc].resize(length_max + 1, -1);

        //assert(CT_paths[loc][timestep] == -1); // should not have vertex conflict
        //assert(timestep == 0 || path[timestep-1].location == -1 || CT_paths[path[timestep-1].location].empty() ||
        //    CT_paths[path[timestep-1].location][timestep] != CT_paths[loc][timestep-1] ||
        //    CT_paths[loc][timestep-1] == -1);  // should not have edge conflict

        CT_paths[loc][timestep] += agent_id + 1;
        latest_conatraints[loc] = std::max(latest_conatraints[loc], timestep);

        // to avoid the situation when an agent cut in line
        if (path[timestep].position_fraction >= 1)
        {
            if (CT_paths[path[timestep].exit_loc].empty())
                CT_paths[path[timestep].exit_loc].resize(length_max + 1, -1);
            if (CT_paths[path[timestep].exit_loc][timestep] < 0)
                latest_conatraints[path[timestep].exit_loc] = std::max(latest_conatraints[path[timestep].exit_loc], timestep);
            CT_paths[path[timestep].exit_loc][timestep] += (1 + agent_id) * num_of_agents + 1;
        }
    }

//    if(malfunction_rate==0){
//        return true;
//    }

//    for (int timestep =0 ; timestep <(int)path.size()-1; timestep++)
//    {
//        int loc = path[timestep].location;
//        //delay estimation correction for insert path before an agent.
//        if(loc == -1 || timestep == (int)path.size()-1){
//            continue;
//        }
//        if(paths_all!= nullptr && timestep+1<=length_max && path[timestep+1].location!= path[timestep].location && path[timestep].delayed_left_time > timestep){
//            for(int t1 = timestep +1; t1<=fmin(path[timestep].delayed_left_time,length_max); t1++){
//                if (CT_paths[loc][t1]>0 && CT_paths[loc][t1]!=agent_id){
//                    int next_agent = CT_paths[loc][t1];
////                    if(t1 >= (*paths_all)[next_agent].size()-1){//for next_agent at goal location;
////                        if (path[timestep].delayed_left_time >= (*paths_all)[next_agent][t1].delayed_left_time){
//////                            float delayed = path[timestep].delayed_left_time - (*paths_all)[next_agent][t1-1].delayed_left_time;
//////                            (*paths_all)[next_agent][t1-1].delayed_left_time = path[timestep].delayed_left_time+delayed* malfunction_rate*36;
////                            (*paths_all)[next_agent][t1].delayed_left_time = path[timestep].delayed_left_time+1;//update goal directly;
//////                            int prev_loc = (*paths_all)[next_agent][t1-1].location;
//////                            int t_prev = t1-2;
//////                            while((*paths_all)[next_agent][t_prev].location == prev_loc){
//////                                (*paths_all)[next_agent][t_prev].delayed_left_time = (*paths_all)[next_agent][t1-1].delayed_left_time;
//////                                t_prev--;
//////                            }
////                        }
////                        continue;
////                    }
//
//                    if (path[timestep].delayed_left_time > (*paths_all)[next_agent][t1-1].delayed_left_time){
//                        float delayed = path[timestep].delayed_left_time
//                                -(*paths_all)[next_agent][t1-1].delayed_left_time;
//                        int prev_loc = (*paths_all)[next_agent][t1-1].location;
//                        int t_prev = t1-1;
//                        while(t_prev >=0 && (*paths_all)[next_agent][t_prev].location == prev_loc){
//                            t_prev--;
//                        }
//
//                        if(delayed>0) {
//                            delay(delayed, next_agent,agent_id, t_prev+1, *paths_all,malfunction_rate);
//                        }
//                    }
//
//                    break;
//                }
//            }
//        }
//    }
    return true;
}

//bool ConstraintTable::delay(float delayed, int next_agent,int origin_agent, int t, vector<Path>& paths_all,float malfunction_rate){
//    float extra_on_node_delay =  delayed * malfunction_rate * 36;
//    paths_all[next_agent][t].on_node_delay+=extra_on_node_delay;
//
//    for(int tp = t; tp < paths_all[next_agent].size(); tp++){
//        paths_all[next_agent][tp].old_left_time = paths_all[next_agent][tp].delayed_left_time;
//        paths_all[next_agent][tp].delayed_left_time+= (delayed +extra_on_node_delay);
//    }
//
//    for(int tp = t; tp < paths_all[next_agent].size(); tp++){
//        int loc = paths_all[next_agent][tp].location;
//        for(int t2 = tp+1; t2<fmin(paths_all[next_agent][t].delayed_left_time,length_max); t2++){
//            if (CT_paths[loc][t2]>0 && CT_paths[loc][t2]!=next_agent ){
//                int new_agent = CT_paths[loc][t2];
//
//                if(paths_all[next_agent][t].delayed_left_time < paths_all[new_agent][t2].latest_left){
//                    break;
//                }
//
//                float next_delayed = (paths_all[next_agent][t].delayed_left_time+paths_all[new_agent][t2].on_node_delay)
//                                     -paths_all[new_agent][t2].delayed_left_time;
//
//                if(next_delayed>0) {
//                    delay(next_delayed, new_agent, origin_agent, t2, paths_all,malfunction_rate);
//                }
//
//
//                break;
//            }
//        }
//    }
//    return true;
//}


bool ConstraintTable::delay(float delayed, int next_agent,int origin_agent, int t, vector<Path>& paths_all,float malfunction_rate){
    float extra_on_node_delay = 0; //  delayed * malfunction_rate * 36;
    for(int tp = t; tp < paths_all[next_agent].size(); tp++){
        if (tp == paths_all[next_agent].size() -1){
            paths_all[next_agent][tp].old_left_time = paths_all[next_agent][tp].delayed_left_time;
            paths_all[next_agent][tp].delayed_left_time = paths_all[next_agent][tp-1].delayed_left_time+1;
//            paths_all[next_agent][tp].delayed_left_time += (delayed);

        }
        else {
            paths_all[next_agent][tp].old_left_time = paths_all[next_agent][tp].delayed_left_time;
            paths_all[next_agent][tp].delayed_left_time += (delayed + extra_on_node_delay);
        }
    }

    //delay update start from time 0 of origin agent to path end. stop recursion to avoid duplicate update
    if(next_agent == origin_agent || paths_all[next_agent][t].location==-1){
        return true;
    }


    int loc = paths_all[next_agent][t].location;


    for(int t2 = t+1; t2<fmin(paths_all[next_agent][t].delayed_left_time,length_max); t2++){
        if (CT_paths[loc][t2]>0 && CT_paths[loc][t2]!=next_agent ){
            int new_agent = CT_paths[loc][t2];


//            if(t2 >= paths_all[new_agent].size()-1){//for new_agent at goal location;
//                if (paths_all[next_agent][t].delayed_left_time >= paths_all[new_agent][t2].delayed_left_time){
//                    paths_all[new_agent][t2].delayed_left_time = paths_all[next_agent][t].delayed_left_time+1;//update goal directly;
//
//                }
//                continue;
//            }



            float next_delayed = paths_all[next_agent][t].delayed_left_time
                                 -paths_all[new_agent][t2-1].delayed_left_time;



            if(next_delayed>0) {
                int prev_loc = paths_all[next_agent][t2-1].location;
                int t_prev = t2-1;
                while(t_prev>=0 && paths_all[new_agent][t_prev].location == prev_loc){
                    t_prev--;
                }
                delay(next_delayed, new_agent, origin_agent, t_prev+1, paths_all,malfunction_rate);
            }


            break;
        }
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
        CT_paths[loc][timestep] -= agent_id + 1;
        if (latest_conatraints[loc] == timestep)
        {
            int t = timestep - 1;
            while(t >= 0 && CT_paths[loc][t] < 0)
                t--;
            latest_conatraints[loc] = t;
        }
        if (path[timestep].position_fraction >= 1)
        {
            CT_paths[path[timestep].exit_loc][timestep] -= (1 + agent_id) * num_of_agents + 1;
            if (CT_paths[path[timestep].exit_loc][timestep] == -1 &&
                latest_conatraints[path[timestep].exit_loc] == timestep)
            {
                int t = timestep - 1;
                while(t >= 0 && CT_paths[path[timestep].exit_loc][t] < 0)
                    t--;
                latest_conatraints[path[timestep].exit_loc] = t;
            }
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

pair<int,int> ConstraintTable::get_previous_agent(int loc, int timestep) const{
    if (loc < 0 || CT_paths[loc].empty())
        return make_pair(-1,0);

    if (timestep > 0){
        for (int t = timestep; t >= 0; t--){
            if (CT_paths[loc][t] >= 0)
                return make_pair(CT_paths[loc][t],t);
        }
    }
    return make_pair(-1,0);
}