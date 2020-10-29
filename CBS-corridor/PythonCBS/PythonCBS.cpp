#include "PythonCBS.h"
#include <time.h>
#include "flat_map_loader.h"
#include <iomanip>
#include <sstream>
#include <math.h>       /* log2 */
#include <algorithm>    // std::random_shuffle
namespace p = boost::python;




template <class Map>
PythonCBS<Map>::PythonCBS(p::object railEnv1, string framework, string algo, float soft_time_limit,
                          int default_group_size, int debug, float f_w, int corridor,bool chasing, bool accept_partial_solution,
                          int agent_priority_strategy, int neighbor_generation_strategy,
                          int prirority_ordering_strategy, int replan_strategy) :
                          railEnv(railEnv1), framework(framework), algo(algo),
                          soft_time_limit(soft_time_limit),
                          f_w(f_w), defaultGroupSize(default_group_size),
                          chasing(chasing),
                          accept_partial_solution(accept_partial_solution),
                          agent_priority_strategy(agent_priority_strategy),
                          neighbor_generation_strategy(neighbor_generation_strategy),
                          prirority_ordering_strategy(prirority_ordering_strategy),
                          replan_strategy(replan_strategy) {
	//Initialize PythonCBS. Load map and agent info into memory
    if (debug)
	    std::cout << "framework: " << framework << "    algo: " << algo << "(" << f_w << ")" << std::endl;
	options1.debug = debug;
    srand(0);
	this->kRobust = 1;
	this->corridor_option = corridor;
    if(corridor == 0){
        this->trainCorridor1 = false;
        this->corridor2 = false;
    }
	else if(corridor == 1){
	    this->trainCorridor1 = true;
	}
	else if(corridor == 2){
	    this->corridor2 = true;
	}
    else if(corridor == 3){
        this->trainCorridor1 = true;
        this->corridor2 = true;
    }
    if (options1.debug)
        cout<<"Corridor option: "<< corridor<< " Chasing option: " << chasing << endl;

	if (algo == "ICBS")
		s = constraint_strategy::ICBS;
	else if (algo == "CBS")
		s = constraint_strategy::CBS;
	else if (algo == "CBSH")
		s = constraint_strategy::CBSH;
	else
	{
		std::cout << "WRONG SOLVER NAME! Use CBSH as default" << std::endl;
		s = constraint_strategy::CBSH;
	}

    if (options1.debug)
	std::cout << "get width height " << std::endl;
	p::long_ rows(railEnv.attr("height"));
	p::long_ cols(railEnv.attr("width"));
    if (options1.debug)
	std::cout << "load map " << p::extract<int>(rows)<<" x "<< p::extract<int>(cols) << std::endl;
	//ml =  new MapLoader(railEnv.attr("rail"), p::extract<int>(rows), p::extract<int>(cols));
    ml = new FlatlandLoader(railEnv.attr("rail"), p::extract<int>(rows), p::extract<int>(cols));
    if (options1.debug)
    std::cout << "load agents " << std::endl;

	al =  new AgentsLoader(railEnv.attr("agents"));
    if (options1.debug)
	std::cout << "load done " << std::endl;
	if (debug) {
		al->printAllAgentsInitGoal();
	}
    al->constraintTable.length_max = p::extract<int>(railEnv.attr("_max_episode_steps"));
    std::cout << "Max timestep = " << al->constraintTable.length_max << endl; // the deadline is stored in the constraint table in al, which will be used for all path finding.
	this->max_malfunction = max_malfunction;

}


template <class Map>
void PythonCBS<Map>::replan(p::object railEnv1, int timestep, float time_limit) {
    int max_timestep = p::extract<int>(railEnv.attr("_max_episode_steps"));
    al->constraintTable.length_max = max_timestep - timestep; // update max timestep
    if (framework == "CPR")
    {
        cpr->planPaths(time_limit);
        return;
    }
    else if (framework == "OnlinePP")
    {
        start_time = Time::now();// time(NULL) return time in seconds
        al->updateAgents(railEnv.attr("agents"));
        for (int a : al->new_agents)
        {
            int start = ml->linearize_coordinate(al->agents_all[a].initial_location);
            opp->to_be_planned_group.push_back(start);
        }
        opp->updateToBePlanedAgents(al->getNumOfAllAgents());
        if (opp->to_be_planned_agents.empty())
            return;

        // use mcp to build new paths
        vector<Path> paths;
        mcp.simulate(paths, timestep);

        runtime = ((fsec)(Time::now() - start_time)).count();
        if (runtime >= time_limit)
            return;

        // update paths
        al->paths_all = paths;
        al->updateConstraintTable();

        runtime = ((fsec)(Time::now() - start_time)).count();
        opp->planPaths(time_limit - runtime);

        mcp.clear();
        mcp.build(al, ml, options1);
        runtime = ((fsec)(Time::now() - start_time)).count();
        cout << "Active agents = " << mcp.active_agents.size() << endl;
        cout << "Runtime for replan  = " << runtime << endl;
        return;
    }
    else // LNS
    {
        start_time = Time::now();// time(NULL) return time in seconds
        al->updateAgents(railEnv.attr("agents"));
        return;
        if (options1.debug)
        {
            cout << "Timestep = " << timestep << ";\t";
            /*cout << "Agent id\t\t: ";
            for (const auto& agent : al->agents_all)
            {
                    cout << agent.agent_id << "\t";
            }
            cout << endl;
            cout << "\t\t\t\tCurr location: ";
            for (const auto& agent : al->agents_all)
            {
                if (agent.status == 1) // active
                    cout << ml->linearize_coordinate(agent.position) << "\t";
                else
                    cout << "-1\t";
            }*/
            cout << endl;
        }
        if (timestep % 100 < 99 || al->new_malfunction_agents.empty())
            return; // replan every 100 timesteps

        if (options1.debug)
        {
            cout << "New mal agent: ";
            for (const auto& agent : al->new_malfunction_agents)
            {
                int loc = -1;
                if (al->agents_all[agent].status == 1)
                    loc = ml->linearize_coordinate(al->agents_all[agent].position);
                cout << agent << " at location " << loc
                     << " (" << al->agents_all[agent].malfunction_left << " timesteps left), ";
            }

            cout << endl;
        }

        // use mcp to build new paths
        vector<Path> paths;
        mcp.simulate(paths, timestep);

        if (options1.debug)
        {
            int old_cost = mcp.getEstimatedCost(timestep);
            int new_cost = 0;
            for (int i = 0 ; i < (int)paths.size(); i++)
            {
                if (al->agents_all[i].status >= 2)  // done (2) or done removed (3)
                    continue;
                else if (paths[i].empty())
                {
                    new_cost += al->constraintTable.length_max;
                }
                else
                {
                    new_cost += (int)paths[i].size() - 1;

                }
            }
            /*cout << "Old paths" << endl;
            for (const auto& path : al->paths_all)
            {
                for (const auto& entry : path)
                    cout << entry.location << "\t";
                cout << endl;
            }
            cout << "New paths" << endl;
            for (const auto& path : paths)
            {
                for (const auto& entry : path)
                    cout << entry.location << "\t";
                cout << endl;
            }*/
            cout << "Cost increase from " << old_cost << " to " << new_cost << "\t";  //endl;
        }
        //if (new_cost - old_cost < 0.01 * al->agents_all.size() * max_timestep)
        //    return; // cost increase is smaller than the threshold

        runtime = ((fsec)(Time::now() - start_time)).count();
        if (runtime >= time_limit)
            return;

        // update paths
        al->paths_all = paths;
        al->updateConstraintTable();

        LNS lns(*al, *ml, f_w, s, agent_priority_strategy, options1, corridor2, trainCorridor1, chasing,
                neighbor_generation_strategy, prirority_ordering_strategy, replan_strategy);
        runtime = ((fsec)(Time::now() - start_time)).count();
        lns.replan(time_limit - runtime);
        // lns.replan(to_be_replanned, time_limit - runtime);
        if (options1.debug)
        {
            //cout << "Updated paths" << endl;
            int new_cost = 0;
            for (int i = 0 ; i < (int)al->agents_all.size(); i++)
            {
                //for (const auto& entry : al->paths_all[i])
                //    cout << entry.location << "\t";
                //cout << endl;
                if (al->agents_all[i].status >= 2) // the agent is done
                    continue;
                if (al->paths_all[i].empty())
                {
                    new_cost += al->constraintTable.length_max;
                }
                else
                {
                    new_cost += (int)al->paths_all[i].size() - 1;
                }
            }
            cout << "Updated cost = " << new_cost << endl;
        }
        mcp.clear();
        mcp.build(al, ml, options1);
        if (options1.debug)
        {
            runtime = ((fsec)(Time::now() - start_time)).count();
            cout << "Runtime = " << runtime << "s." << endl;
        }
        al->new_malfunction_agents.clear();
    }
}

template <class Map>
void PythonCBS<Map>::updateFw(float fw) {
	f_w = fw;
}


template <class Map>
p::list PythonCBS<Map>::getResult() {
	//output current paths
	return al->outputPaths();
}

template <class Map>
bool PythonCBS<Map>::search() {
    start_time = Time::now();// time(NULL) return time in seconds
    if (options1.debug)
		cout << "start initialize" << endl;
	//initialize search engine
	al->constraintTable.init(ml->map_size());
    al->computeHeuristics(ml);
    this->statistic_list.resize(1);
    this->iteration_stats.resize(1);
    if (framework == "OnlinePP")
    {
        runtime = ((fsec)(Time::now() - start_time)).count();
        opp = new OnlinePP(*al, *ml, options1, soft_time_limit - runtime);
        return true;
    }
    else if (framework == "CPR")
    {
        runtime = ((fsec)(Time::now() - start_time)).count();
        cpr = new CPR(*al, *ml, options1, hard_time_limit - runtime);
        return true;
    }
    else if (framework == "LNS")
    {
        LNS lns(*al, *ml, f_w, s, agent_priority_strategy, options1, corridor2, trainCorridor1, chasing,
                neighbor_generation_strategy, prirority_ordering_strategy, replan_strategy);
        runtime = ((fsec)(Time::now() - start_time)).count(); 
        bool succ = lns.run(hard_time_limit - runtime, soft_time_limit - runtime);
        runtime = ((fsec)(Time::now() - start_time)).count();
        statistic_list[0].HL_num_expanded = lns.HL_num_expanded;
        statistic_list[0].HL_num_generated = lns.HL_num_generated;
        statistic_list[0].LL_num_expanded = lns.LL_num_expanded;
        statistic_list[0].LL_num_generated = lns.LL_num_generated;
        statistic_list[0].num_standard = lns.num_standard;
        statistic_list[0].num_chasing = lns.num_chasing;
        statistic_list[0].num_start = lns.num_start;
        statistic_list[0].num_corridor = lns.num_corridor;
        statistic_list[0].num_corridor2 = lns.num_corridor2;
        statistic_list[0].runtime_corridor = lns.runtime_corridor;
        iteration_stats[0] = lns.iteration_stats;
        if (findConflicts()){
            assert("Find conflict.");
        }

        return succ;
    }
    else if(framework == "Parallel-LNS")
        return parallel_LNS(4);
    else if(framework == "Parallel-Neighbour-LNS")
        return parallel_neighbour_LNS(4);
    else
	    return false;
}


template <class Map>
bool PythonCBS<Map>::findConflicts() const
{
    for (int i = 0; i < (int)al->agents_all.size() - 1; i++)
    {
        for (int j = i + 1; j < (int)al->agents_all.size(); j++)
        {
            for (int t = 0; t < (int)al->paths_all[i].size(); t++)
            {
                int loc = al->paths_all[i][t].location;
                if (loc < 0)
                    continue;
                if (t < al->paths_all[j].size() && al->paths_all[j][t].location == loc)
                    return true;
                if (t < al->paths_all[j].size() && t>=1
                && al->paths_all[j][t-1].location == loc &&  al->paths_all[j][t].location == al->paths_all[i][t-1].location)
                    return true;

            }
        }
    }
    return false;
}

template <class Map>
p::dict PythonCBS<Map>::getResultDetail() {
	//return result detail
	p::dict result;
	int thread_id = this->best_thread_id;

	result["runtime"] = runtime;
	result["HL_expanded"] = statistic_list[thread_id].HL_num_expanded;
	result["HL_generated"] = statistic_list[thread_id].HL_num_generated;
	result["LL_expanded"] = statistic_list[thread_id].LL_num_expanded;
	result["LL_generated"] = statistic_list[thread_id].LL_num_generated;
    std::stringstream stream;
    stream << std::fixed << std::setprecision(1) << f_w;
    std::string s = stream.str();
	result["algorithm"] = framework +
            to_string(neighbor_generation_strategy) + to_string(prirority_ordering_strategy) + to_string(replan_strategy) +
            "_" + algo + "(" + s + ")_groupsize=" + to_string(defaultGroupSize) +
	        "_priority=" + to_string(agent_priority_strategy);
    result["num_standard"] = statistic_list[thread_id].num_standard;
    result["num_chasing"] = statistic_list[thread_id].num_chasing;
    result["num_start"] = statistic_list[thread_id].num_start;
    result["num_corridor"] = statistic_list[thread_id].num_corridor;
    result["num_corridor2"] = statistic_list[thread_id].num_corridor2;
    result["runtime_corridor"] = statistic_list[thread_id].runtime_corridor;

    result["best_initial_priority_strategy"] = this->best_initisl_priority_strategy;
    result["best_neighboour_strategy"] = this->best_neighbour_strategy;



    int solution_cost = 0;
    int finished_agents = 0;
    int makespan = 0;
    for (const auto& path : al->paths_all)
    {
        solution_cost += (int)path.size() - 1;
        makespan = max((int)path.size() - 1, makespan);
        if (!path.empty())
            finished_agents++;
    }
    result["solution_cost"] = solution_cost;
    result["finished_agents"] = finished_agents;
    result["makespan"] = makespan;
	return result;
}

template <class Map>
void PythonCBS<Map>::generateNeighbor(int agent_id, const PathEntry& start, int start_time, set<int>& conflicting_agents, int neighbor_size, int upperbound)
{
    // a random walk with path that is shorter than upperbound and has conflicting with neighbor_size agents
    int speed = al->agents_all[agent_id].speed;
    const auto& heuristics = *al->agents_all[agent_id].heuristics;
    int loc = start.location;
    int heading = start.heading;
    auto position_fraction = start.position_fraction;
    auto exit_heading = start.exit_heading;
    int exit_loc = start.exit_loc;
    int h_val = heuristics[loc].get_hval(heading) / speed;
    if (exit_loc >= 0 && speed < 1)
    {
        int h1 = heuristics[loc].get_hval(heading);
        int h2 = heuristics[exit_loc].get_hval(exit_heading);
        h_val = h1 / speed - (h2 - h1)*position_fraction;
    }

    for (int t = start_time; t < upperbound; t++)
    {
        list<Transition> transitions;
        if(loc == -1){
            Transition move;
            move.location = loc;
            move.heading = heading;
            move.position_fraction = position_fraction;
            move.exit_loc = exit_loc;
            move.exit_heading = exit_heading;
            transitions.push_back(move);

            Transition move2;
            move2.location = ml->linearize_coordinate(al->agents_all[agent_id].position.first, al->agents_all[agent_id].position.second);
            move2.heading = heading;
            move2.position_fraction = position_fraction;
            move2.exit_loc = exit_loc;
            move2.exit_heading = exit_heading;
            transitions.push_back(move2);
        }
        else if (position_fraction + speed >= 0.97)
        {
            if (position_fraction == 0)
            {
                ml->get_transitions(transitions, loc, heading, false);
                assert(!transitions.empty());
            }
            else {
                Transition move;
                move.location = exit_loc;
                move.heading = exit_heading;
                move.position_fraction = 0;
                transitions.push_back(move);
            }
        }
        else if (position_fraction == 0)
        {
            ml->get_exits(transitions, loc, heading, speed, false);
            assert(!transitions.empty());
        }
        else { //<0.97 and po_frac not 0
            Transition move2;
            move2.location = loc;
            move2.heading = heading;
            move2.position_fraction = position_fraction + al->agents[agent_id]->speed;
            move2.exit_loc = exit_loc;
            move2.exit_heading = exit_heading;
            transitions.push_back(move2);
        }

        while(!transitions.empty())
        {
            int step = rand() % transitions.size();
            auto it = transitions.begin();
            advance(it, step);
            int next_h_val;
            if (it->location == -1)
                next_h_val = h_val;
            else if (exit_loc >= 0 && speed < 1)
            {
                int h1 = heuristics[it->location].get_hval(it->heading);
                int h2 = heuristics[it->exit_loc].get_hval(it->exit_heading);
                next_h_val = h1 / speed - (h2 - h1) * (it->position_fraction / speed);
            }
            else
                next_h_val = heuristics[it->location].get_hval(it->heading) / speed;

            if (t + 1 + next_h_val < upperbound) // move to this location
            {
                loc = it->location;
                heading = it->heading;
                position_fraction = it->position_fraction;
                exit_heading = it->exit_heading;
                exit_loc = it->exit_loc;
                h_val = next_h_val;
                assert(agent_id == al->agents_all[agent_id].agent_id);
                al->constraintTable.get_conflicting_agents(agent_id, conflicting_agents, loc, t + 1);
                break;
            }
            transitions.erase(it);
        }
        if (transitions.empty() || conflicting_agents.size() >= neighbor_size || h_val == 0)
            break;
    }
}

template <class Map>
bool PythonCBS<Map>::parallel_LNS(int no_threads){
    al->constraintTable.init(ml->map_size());
    al->computeHeuristics(ml);
    this->al_pool.resize(no_threads);
    this->lns_pool.resize(no_threads);
    this->statistic_list.resize(no_threads);
    this->iteration_stats.resize(no_threads);
    pthread_t threads[no_threads];
    if (no_threads>4){
        cout<<"Max threads number: 4"<<endl;
        exit(1);
    }
    for (int i = 0; i<no_threads;i++){
        AgentsLoader* temp = this->al->clone();
        this->al_pool[i] = temp;
        this->lns_pool[i] = new LNS(*al_pool[i], *ml, f_w, s, strategies[i], options1, corridor2, trainCorridor1, chasing,
                neighbor_generation_strategy, prirority_ordering_strategy, replan_strategy);
        runtime = ((fsec)(Time::now() - start_time)).count();;
        wrap* w = new wrap(hard_time_limit - runtime, soft_time_limit - runtime,*this->lns_pool[i]);
        pthread_create( &threads[i], NULL, call_func, w );
    }
    // wait until all finish
    for (int i = 0; i<no_threads;i++){
        pthread_join(threads[i], NULL);
    }

    int best_cost = INT_MAX;
    int best_al = -1;
    int best_makespan = -1;
    int best_finished_agents = -1;
    for (int i=0; i<no_threads; i++){
        statistic_list[i].HL_num_expanded = lns_pool[i]->HL_num_expanded;
        statistic_list[i].HL_num_generated = lns_pool[i]->HL_num_generated;
        statistic_list[i].LL_num_expanded = lns_pool[i]->LL_num_expanded;
        statistic_list[i].LL_num_generated = lns_pool[i]->LL_num_generated;
        statistic_list[i].num_standard = lns_pool[i]->num_standard;
        statistic_list[i].num_chasing = lns_pool[i]->num_chasing;
        statistic_list[i].num_start = lns_pool[i]->num_start;
        statistic_list[i].num_corridor = lns_pool[i]->num_corridor;
        statistic_list[i].num_corridor2 = lns_pool[i]->num_corridor2;
        statistic_list[i].runtime_corridor = lns_pool[i]->runtime_corridor;
        iteration_stats[i] = lns_pool[i]->iteration_stats;

        size_t solution_cost = 0;
        int finished_agents = 0;
        size_t makespan = 0;
        for (const auto& path : this->al_pool[i]->paths_all)
        {
            solution_cost += path.size();
            makespan = max(path.size(), makespan);
            if (!path.empty())
                finished_agents++;
        }
        if (options1.debug){
            cout <<"Initial PP strategy = "<<strategies[i]<<", "
                << "Final Solution cost = " << solution_cost << ", "
                << "Final makespan = " << makespan  << ", "
                << endl;
        }
        if (solution_cost < best_cost){
            best_cost = solution_cost;
            best_al = i;
            best_makespan = makespan;
            best_finished_agents = finished_agents;
        }
    }
    delete this->al;
    this->al = this->al_pool[best_al];
    this->best_thread_id = best_al;
    this->best_initisl_priority_strategy = strategies[best_al];
    runtime = ((fsec)(Time::now() - start_time)).count();;
    if (options1.debug) {
        cout << "Best PP strategy = " << strategies[best_al] << ", "
             << "Final Solution cost = " << best_cost << ", "
             << "Final makespan = " << best_makespan << ", "
             << "Final runtime = " << runtime << endl;
    }
    return true;
}

template <class Map>
bool PythonCBS<Map>::parallel_neighbour_LNS(int no_threads){
    al->constraintTable.init(ml->map_size());
    al->computeHeuristics(ml);
    this->al_pool.resize(no_threads);
    this->lns_pool.resize(no_threads);
    this->statistic_list.resize(no_threads);
    this->iteration_stats.resize(no_threads);
    pthread_t threads[no_threads];
    vector<wrap*> wrap_handle;
    if (no_threads>4){
        cout<<"Max threads number: 4"<<endl;
        exit(1);
    }
    //run pp only
    for (int i = 0; i<no_threads;i++){
        AgentsLoader* temp = this->al->clone();
        this->al_pool[i] = temp;
        this->lns_pool[i] = new LNS(*al_pool[i], *ml, f_w, s, strategies[i], options1, corridor2, trainCorridor1, chasing,
                                    3, prirority_ordering_strategy, replan_strategy);
        this->lns_pool[i]->pp_only = true;
        runtime = ((fsec)(Time::now() - start_time)).count();;
        wrap* w = new wrap(hard_time_limit - runtime, soft_time_limit - runtime,*this->lns_pool[i]);
        wrap_handle.push_back(w);
        pthread_create( &threads[i], NULL, call_func, w );
    }
    // wait until all pp finish
    for (int i = 0; i<no_threads;i++){
        pthread_join(threads[i], NULL);
    }

    int best_pp_cost = INT_MAX;
    int best_pp_al = 0;

    for (int i=0; i<no_threads; i++){

        size_t solution_cost = 0;
        int finished_agents = 0;
        size_t makespan = 0;
        for (const auto& path : this->al_pool[i]->paths_all)
        {
            solution_cost += path.size();
            makespan = max(path.size(), makespan);
            if (!path.empty())
                finished_agents++;
        }
        if (options1.debug){
            cout <<"Initial PP strategy = "<<strategies[i]<<", "
                 << "Final Solution cost = " << solution_cost << ", "
                 << "Final makespan = " << makespan  << ", "
                 << endl;
        }
        if (solution_cost < best_pp_cost){
            best_pp_cost = solution_cost;
            best_pp_al = i;
        }
    }
    this->best_initisl_priority_strategy = strategies[best_pp_al];

    //copy best pp al
    for (int i = 0; i<no_threads;i++) {
        if (i ==best_pp_al)
            continue;
        AgentsLoader *temp = this->al_pool[best_pp_al]->clone();
        delete this->al_pool[i];
        this->al_pool[i] = temp;
    }

    //run lns skip pp
    for (int i = 0; i<no_threads;i++){
        delete this->lns_pool[i];
        this->lns_pool[i] = new LNS(*al_pool[i], *ml, f_w, s, 0, options1, corridor2, trainCorridor1, chasing,
                                    neighbours[i], prirority_ordering_strategy, replan_strategy);
        this->lns_pool[i]->skip_pp = true;
        runtime = ((fsec)(Time::now() - start_time)).count();;
        wrap* w = new wrap(hard_time_limit - runtime, soft_time_limit - runtime,*this->lns_pool[i]);
        wrap_handle.push_back(w);
        pthread_create( &threads[i], NULL, call_func, w );
    }
    // wait until all finish
    for (int i = 0; i<no_threads;i++){
        pthread_join(threads[i], NULL);
    }

    int best_cost = INT_MAX;
    int best_al = -1;
    int best_makespan = -1;
    int best_finished_agents = -1;
    for (int i=0; i<no_threads; i++){
        statistic_list[i].HL_num_expanded = lns_pool[i]->HL_num_expanded;
        statistic_list[i].HL_num_generated = lns_pool[i]->HL_num_generated;
        statistic_list[i].LL_num_expanded = lns_pool[i]->LL_num_expanded;
        statistic_list[i].LL_num_generated = lns_pool[i]->LL_num_generated;
        statistic_list[i].num_standard = lns_pool[i]->num_standard;
        statistic_list[i].num_chasing = lns_pool[i]->num_chasing;
        statistic_list[i].num_start = lns_pool[i]->num_start;
        statistic_list[i].num_corridor = lns_pool[i]->num_corridor;
        statistic_list[i].num_corridor2 = lns_pool[i]->num_corridor2;
        statistic_list[i].runtime_corridor = lns_pool[i]->runtime_corridor;
        iteration_stats[i] = lns_pool[i]->iteration_stats;

        size_t solution_cost = 0;
        int finished_agents = 0;
        size_t makespan = 0;
        for (const auto& path : this->al_pool[i]->paths_all)
        {
            solution_cost += path.size();
            makespan = max(path.size(), makespan);
            if (!path.empty())
                finished_agents++;
        }
        if (options1.debug){
            cout <<"Initial PP strategy = "<<strategies[i]<<", "
                 << "Final Solution cost = " << solution_cost << ", "
                 << "Final makespan = " << makespan  << ", "
                 << endl;
        }
        if (solution_cost < best_cost){
            best_cost = solution_cost;
            best_al = i;
            best_makespan = makespan;
            best_finished_agents = finished_agents;
        }
    }
    delete this->al;
    this->al = this->al_pool[best_al];
    this->best_thread_id = best_al;
    this->best_neighbour_strategy = neighbours[best_al];
    runtime = ((fsec)(Time::now() - start_time)).count();;
    if (options1.debug) {
        cout << "Best PP strategy = " << strategies[best_al] << ", "
             << "Final Solution cost = " << best_cost << ", "
             << "Final makespan = " << best_makespan << ", "
             << "Final runtime = " << runtime << endl;
    }

    return true;
}

template <class Map>
void PythonCBS<Map>::updateAgents(p::object railEnv1){
    al->updateAgents(railEnv.attr("agents"));
}

template class PythonCBS<FlatlandLoader>;

BOOST_PYTHON_MODULE(libPythonCBS)  // Name here must match the name of the final shared library, i.e. mantid.dll or mantid.so
{
	using namespace boost::python;
	class_<PythonCBS<FlatlandLoader>>("PythonCBS", init<object, string, string, float,
	        int, int,float,int,bool,bool,int,int,int,int>())
		.def("getResult", &PythonCBS<FlatlandLoader>::getResult)
		.def("search", &PythonCBS<FlatlandLoader>::search)
		.def("hasConflicts", &PythonCBS<FlatlandLoader>::findConflicts)
		.def("getResultDetail", &PythonCBS<FlatlandLoader>::getResultDetail)
		.def("writeResultsToFile", &PythonCBS<FlatlandLoader>::writeResultsToFile)
		.def("replan",&PythonCBS<FlatlandLoader>::replan)
		.def("updateAgents", &PythonCBS<FlatlandLoader>::updateAgents)
		.def("updateFw", &PythonCBS<FlatlandLoader>::updateFw)
		.def("buildMCP", &PythonCBS<FlatlandLoader>::buildMCP)
        .def("getNextLoc", &PythonCBS<FlatlandLoader>::getNextLoc)
        .def("updateMCP", &PythonCBS<FlatlandLoader>::updateMCP)
        .def("clearMCP", &PythonCBS<FlatlandLoader>::clearMCP)
        .def("printAllMCP", &PythonCBS<FlatlandLoader>::printAllMCP)
        .def("printMCP", &PythonCBS<FlatlandLoader>::printMCP)
        .def("printAgentTime", &PythonCBS<FlatlandLoader>::printAgentTime)
        .def("printAgentNoWaitTime", &PythonCBS<FlatlandLoader>::printAgentNoWaitTime);
}