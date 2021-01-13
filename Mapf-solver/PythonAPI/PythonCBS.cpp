/*
 * @author: Team An_old_driver
 * @created: 09-2020
 * Copyright (c) 2020 The University of Southern California. All Rights Reserved.
 * Copyrights licensed under an Academic/non-profit use license.
 * See the accompanying LICENSE file for terms.
*/

#include "PythonCBS.h"
#include <time.h>
#include "flat_map_loader.h"
#include <iomanip>
#include <sstream>
#include <math.h>       /* log2 */
#include <algorithm>    // std::random_shuffle
namespace p = boost::python;




template <class Map>
PythonCBS<Map>::PythonCBS(p::object railEnv1, string framework, float soft_time_limit,
                          int default_group_size, bool debug, bool replan, int stop_threshold, int agent_priority_strategy, int neighbor_generation_strategy) :
                          railEnv(railEnv1), framework(framework), soft_time_limit(soft_time_limit),
                          default_group_size(default_group_size), replan_on(replan),stop_threshold(stop_threshold),agent_priority_strategy(),neighbor_generation_strategy() {
	//Initialize PythonCBS. Load map and agent info into memory
	hard_time_limit = soft_time_limit;
    if (debug)
	    std::cout << "framework: " << framework << std::endl;
	options1.debug = debug;
    srand(0);

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

	al =  new AgentsLoader(*ml, railEnv.attr("agents"));
    if (options1.debug)
	    std::cout << "load done " << std::endl;
    max_timestep = p::extract<int>(railEnv.attr("_max_episode_steps"));
    al->constraintTable.length_max = max_timestep;
    std::cout << "Max timestep = " << max_timestep << endl; // the deadline is stored in the constraint table in al, which will be used for all path finding.
    malfunction_rate = p::extract<float>(railEnv.attr("malfunction_process_data").attr("malfunction_rate"));
    std::cout << "Malfunction rate = " << malfunction_rate << std::endl;
    curr_locations.resize(ml->map_size(), -1);
    prev_locations.resize(ml->map_size(), -1);
    action_converter.num_agent = al->getNumOfAllAgents();
    action_converter.env_width = ml->cols;
}

template <class Map>
p::dict PythonCBS<Map>::getActions(p::object railEnv1, int timestep, float time_limit)
{

    replan(railEnv1, timestep, time_limit);// replan

    // update curr and prev locations
    for (int i = 0; i < al->getNumOfAllAgents(); i++)
    {
        if (curr_locations[i] != al->agents_all[i].position)
            prev_locations[i] = curr_locations[i];
        curr_locations[i] = al->agents_all[i].position;
    }

    boost::python::dict actions;
    if (framework == "CPR")
    {
        if (options1.debug)
            cout << "Timestep " << timestep << endl;
        cpr->getNextLoc();
        // convert next location to actions of boost dict structure
        for(int i =0; i<al->getNumOfAllAgents(); i++){
            actions[i] = action_converter.pos2action(curr_locations[i], prev_locations[i], cpr->to_go[i]);
        }
    }
    else
    {
        mcp.getNextLoc(timestep +  1); // get next location
        // convert next location to actions of boost dict structure
        for(int i =0; i<al->getNumOfAllAgents(); i++){
            actions[i] = action_converter.pos2action(curr_locations[i], prev_locations[i], mcp.to_go[i]);
        }
    }
    return actions;
}

template <class Map>
void PythonCBS<Map>::replan(const p::object& railEnv1, int timestep, float time_limit) {
    start_time = Time::now();// time(NULL) return time in seconds
    int _max_timestep = p::extract<int>(railEnv.attr("_max_episode_steps"));
    al->constraintTable.length_max = _max_timestep - timestep; // update max timestep
    al->updateAgents(*ml, railEnv.attr("agents"));
    if (framework == "CPR")
    {
        cpr->update();
        cpr->planPaths(time_limit);
        return;
    }
    else // LNS
    {
        // TODO: for malfunction agents who have not entered the environment, we can simply replan the paths for themselves.
        mcp.update();
        if (!al->unplanned_agents.empty() && timestep >= 500 && timestep % 100 == 0)
        {
            auto remaining_agents = al->unplanned_agents.size();
            vector<Path> paths;
            mcp.simulate(paths, timestep);
            al->paths_all = paths;
            al->updateConstraintTable();

            LNS lns(*al, *ml, 1, agent_priority_strategy, options1, default_group_size,
                    neighbor_generation_strategy, prirority_ordering_strategy, replan_strategy,this->stop_threshold);
            runtime = ((fsec)(Time::now() - start_time)).count();
            lns.replan(al->unplanned_agents, 2 * time_limit - runtime);
            mcp.clear();
            mcp.build(al, ml, options1);
            runtime = ((fsec)(Time::now() - start_time)).count();
            replan_runtime += runtime;
            //cout << "Timestep " << timestep << ": Plan paths for " <<
            //    remaining_agents - al->unplanned_agents.size() << " agents using " << runtime << "seconds" << endl;
            return;
        }
        if (!replan_on || replan_times >= max_replan_times  || replan_runtime >= max_replan_runtime)
            return;

        for (auto p = al->new_malfunction_agents.begin(); p != al->new_malfunction_agents.end();)
        {
            // if the agent always wait during the malfunction duration, then we do not need to replan
            if (al->agents_all[*p].status == 0 && al->agents_all[*p].malfunction_left + timestep <= mcp.appear_time[*p])
                p = al->new_malfunction_agents.erase(p);
            else
                ++p;
        }
        if (al->new_malfunction_agents.empty() && to_be_replanned.empty())
            return; // we do not replan if there are no new mal agents and no to-be-replanned agents

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
//        if (timestep % 100 < 99 || al->new_malfunction_agents.empty())
//            return; // replan every 100 timesteps

        if (options1.debug)
        {
            cout << "New mal agent: ";
            for (const auto& agent : al->new_malfunction_agents)
            {
                int loc = -1;
                if (al->agents_all[agent].status == 1)
                    loc = al->agents_all[agent].position;
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
        {
            replan_runtime += runtime;
            return;
        }

        // update paths
        al->paths_all = paths;
        al->updateConstraintTable();

        LNS lns(*al, *ml, 1, agent_priority_strategy, options1, default_group_size,
                neighbor_generation_strategy, prirority_ordering_strategy, replan_strategy,this->stop_threshold);
        runtime = ((fsec)(Time::now() - start_time)).count();
        lns.replan(time_limit - runtime);
        replan_times += lns.replan_times;
        //if (lns.dead_agent) // when we know for sure that we will not make it
        //    replan_on = false; // give up!
//         lns.replan(to_be_replanned, time_limit - runtime);
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
        replan_runtime += ((fsec)(Time::now() - start_time)).count();
    }
}


template <class Map>
p::list PythonCBS<Map>::getResult() {
	//output current paths
	return al->outputPaths();
}

template <class Map>
bool PythonCBS<Map>::search(float success_rate, int max_iterations) {
    start_time = Time::now();// time(NULL) return time in seconds
    if (options1.debug)
		cout << "start initialize" << endl;
	//initialize search engine
	al->constraintTable.init(ml->map_size());
    al->computeHeuristics(ml, this->existing_heuristics);
    if (options1.debug) {
        al->printAllAgentsInitGoal();
    }
    this->statistic_list.resize(1);
    this->iteration_stats.resize(1);
    if (framework == "CPR")
    {
        runtime = ((fsec)(Time::now() - start_time)).count();
        cpr = new CPR(*al, *ml, options1, hard_time_limit - runtime);
        return true;
    }
    else if (framework == "LNS")
    {
        LNS lns(*al, *ml, 1, agent_priority_strategy, options1, default_group_size,
                neighbor_generation_strategy, prirority_ordering_strategy, replan_strategy,this->stop_threshold);
        runtime = ((fsec)(Time::now() - start_time)).count(); 
        bool succ = lns.run(hard_time_limit - runtime, soft_time_limit - runtime, success_rate, max_iterations);
        if (!succ && malfunction_rate < 0) // fail to plan paths for all agents for no-mal instances
        {
            for (int i = (int) lns.neighbors.size() - 1; i >= 0; i--)
            {
                if (!al->paths_all[lns.neighbors[i]].empty())
                    break;
                al->unplanned_agents.push_front(lns.neighbors[i]);
            }
        }
        runtime = ((fsec)(Time::now() - start_time)).count();
        statistic_list[0].runtime = runtime;
        statistic_list[0].initial_runtime = lns.initial_runtime;
        statistic_list[0].sum_of_costs = lns.sum_of_costs;
        statistic_list[0].initial_sum_of_costs = lns.initial_sum_of_costs;
        statistic_list[0].makespan = lns.makespan;
        statistic_list[0].initial_makespan = lns.initial_makespan;
        statistic_list[0].iterations = lns.iterations;
        iteration_stats[0] = lns.iteration_stats;
        if (options1.debug && findConflicts()){
            assert("Find conflict.");
        }

        return succ;
    }
    else if(framework == "Parallel-LNS")
        return parallel_LNS(4, success_rate, max_iterations);
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

	result["initial_runtime"] = statistic_list[thread_id].initial_runtime;
	result["final_runtime"] = statistic_list[thread_id].runtime;
	result["initial_sum_of_costs"] = statistic_list[thread_id].initial_sum_of_costs;
	result["final_sum_of_costs"] = statistic_list[thread_id].sum_of_costs;
	result["initial_makespan"] = statistic_list[thread_id].initial_makespan;
    result["final_makespan"] = statistic_list[thread_id].makespan;
    result["initial_reward"] = statistic_list[thread_id].initial_sum_of_costs * 1.0 /
            (max_timestep * al->getNumOfAllAgents());
    result["final_reward"] = statistic_list[thread_id].sum_of_costs * 1.0 /
            (max_timestep * al->getNumOfAllAgents());
    result["iterations"] = statistic_list[thread_id].iterations;
    result["replan_times"] = replan_times;
    result["num_of_agents"] = al->getNumOfAllAgents();
    result["max_timestep"] = max_timestep;
    result["malfunction_rate"] = malfunction_rate;
    result["unplanned agents"] =statistic_list[thread_id].unfinished_agents;
    //std::stringstream stream;
    //stream << std::fixed << std::setprecision(1) << f_w;
    //std::string s = stream.str();
	//result["algorithm"] = framework +
    //        to_string(neighbor_generation_strategy) + to_string(prirority_ordering_strategy) + to_string(replan_strategy) +
    //        "_" + algo + "(" + s + ")_groupsize=" + to_string(defaultGroupSize) +
	//        "_priority=" + to_string(agent_priority_strategy);

    //result["best_initial_priority_strategy"] = this->best_initisl_priority_strategy;
    //result["best_neighboour_strategy"] = this->best_neighbour_strategy;



    /*int solution_cost = 0;
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
    result["makespan"] = makespan;*/
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
            move2.location = al->agents_all[agent_id].position;
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
bool PythonCBS<Map>::parallel_LNS(int no_threads, float success_rate, int max_iterations){
    al->constraintTable.init(ml->map_size());
    al->computeHeuristics(ml,this->existing_heuristics);
    this->al_pool.resize(no_threads);
    this->lns_pool.resize(no_threads);
    this->statistic_list.resize(no_threads);
    this->iteration_stats.resize(no_threads);
    pthread_t threads[no_threads];
    if (no_threads>4){
        cout<<"Max threads number: 4"<<endl;
        exit(1);
    }
    std::atomic<int> complete;
    complete.store(-MAX_COST);

    for (int i = 0; i<no_threads;i++){
        AgentsLoader* temp = this->al->clone();
        this->al_pool[i] = temp;
        this->lns_pool[i] = new LNS(*al_pool[i], *ml, 1, strategies[i], options1,
                default_group_size, neighbor_generation_strategy, prirority_ordering_strategy, replan_strategy,this->stop_threshold);
        this->lns_pool[i]->set_complete(&complete);
        runtime = ((fsec)(Time::now() - start_time)).count();
        wrap* w = new wrap(hard_time_limit - runtime, soft_time_limit - runtime,
                success_rate, max_iterations,*this->lns_pool[i]);
        pthread_create( &threads[i], NULL, call_func, w );
    }
    // wait until all finish
    for (int i = 0; i<no_threads;i++){
        pthread_join(threads[i], NULL);
    }
    runtime = ((fsec)(Time::now() - start_time)).count();

    int best_cost = INT_MAX;
    int best_al = -1;
    int best_makespan = -1;
    int best_finished_agents = -1;
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
        if ((best_finished_agents < finished_agents) ||
            (best_finished_agents == finished_agents && solution_cost < best_cost)){
            best_cost = solution_cost;
            best_al = i;
            best_makespan = makespan;
            best_finished_agents = finished_agents;
        }
        statistic_list[i].runtime = runtime;
        statistic_list[i].initial_runtime = this->lns_pool[i]->initial_runtime;
        statistic_list[i].sum_of_costs = this->lns_pool[i]->sum_of_costs;
        statistic_list[i].initial_sum_of_costs = this->lns_pool[i]->initial_sum_of_costs;
        statistic_list[i].makespan = this->lns_pool[i]->makespan;
        statistic_list[i].initial_makespan = this->lns_pool[i]->initial_makespan;
        statistic_list[i].iterations = this->lns_pool[i]->iterations;
        iteration_stats[i] = this->lns_pool[i]->iteration_stats;


    }
    delete this->al;
    this->al = this->al_pool[best_al];
    this->best_thread_id = best_al;
    this->best_initisl_priority_strategy = strategies[best_al];
    if (best_finished_agents < al->getNumOfAllAgents() && malfunction_rate < 0) // fail to plan paths for all agents for small-mal instances
    {
        for (int i = 0; i < (int) al->paths_all.size(); i++)
        {
            if (al->paths_all[i].empty())
                al->unplanned_agents.push_front(i);
        }
    }
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
    al->updateAgents(*ml, railEnv.attr("agents"));
}

template class PythonCBS<FlatlandLoader>;

BOOST_PYTHON_MODULE(libPythonCBS)  // Name here must match the name of the final shared library, i.e. mantid.dll or mantid.so
{
	using namespace boost::python;
	class_<PythonCBS<FlatlandLoader>>("PythonCBS", init<object, string, float,int, bool, bool,int,int,int>())
		.def("getResult", &PythonCBS<FlatlandLoader>::getResult)
		.def("search", &PythonCBS<FlatlandLoader>::search)
		.def("hasConflicts", &PythonCBS<FlatlandLoader>::findConflicts)
		.def("getResultDetail", &PythonCBS<FlatlandLoader>::getResultDetail)
		.def("writeResultsToFile", &PythonCBS<FlatlandLoader>::writeResultsToFile)
		.def("updateAgents", &PythonCBS<FlatlandLoader>::updateAgents)
		.def("buildMCP", &PythonCBS<FlatlandLoader>::buildMCP)
        .def("getActions",&PythonCBS<FlatlandLoader>::getActions)
        .def("clearMCP", &PythonCBS<FlatlandLoader>::clearMCP)
        .def("printAllMCP", &PythonCBS<FlatlandLoader>::printAllMCP)
        .def("printMCP", &PythonCBS<FlatlandLoader>::printMCP)
        .def("printAgentTime", &PythonCBS<FlatlandLoader>::printAgentTime)
        .def("printAgentNoWaitTime", &PythonCBS<FlatlandLoader>::printAgentNoWaitTime);
}