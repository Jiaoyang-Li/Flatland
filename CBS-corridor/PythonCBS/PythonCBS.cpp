#include "PythonCBS.h"
#include <time.h>
#include "flat_map_loader.h"
#include "MDD.h"
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
void PythonCBS<Map>::updateAgents(p::object railEnv1) {
	if (options1.debug)
		cout << "update Agents" << endl;
	al->updateAgents(railEnv.attr("agents"));
	
	//if (icbs != NULL)
	//	delete icbs;
	if (options1.debug)
		cout << "update Agents done!" << endl;
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
    start_time = time(NULL);// time(NULL) return time in seconds
    if (options1.debug)
		cout << "start initialize" << endl;
	//initialize search engine
	al->constraintTable.init(ml->map_size());
    al->computeHeuristics(ml);
    this->statistic_list.resize(1);
    this->iteration_stats.resize(1);
    if (framework == "LNS")
    {
        LNS lns(*al, *ml, f_w, s, agent_priority_strategy, options1, corridor2, trainCorridor1, chasing,
                neighbor_generation_strategy, prirority_ordering_strategy, replan_strategy);
        runtime = (double)(time(NULL)- start_time) ;
        bool succ = lns.run(hard_time_limit - runtime, soft_time_limit - runtime);
        runtime = (double)(time(NULL) - start_time);
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
        return succ;
    }
    else if(framework == "Parallel-LNS")
        return parallel_LNS(4);
    else
	    return GroupPrioritizedPlaning();
}


template <class Map>
bool PythonCBS<Map>::findConflicts() const
{
    assert(kRobust > 0); // TODO: consider kDelay==0 in the future (in which case, we also need to consider edge conflicts)
    for (int i = 0; i < (int)al->agents_all.size() - 1; i++)
    {
        for (int j = i + 1; j < (int)al->agents_all.size(); j++)
        {
            for (int t = 0; t < (int)al->paths_all[i].size(); t++)
            {
                int loc = al->paths_all[i][t].location;
                if (loc < 0)
                    continue;
                for (int timestep = max(0, t - kRobust); timestep <= min(t + kRobust, (int)al->paths_all[j].size() - 1); timestep++)
                {
                    if (al->paths_all[j][timestep].location == loc)
                        return true;
                }
            }
        }
    }
    return false;
}


template <class Map>
bool PythonCBS<Map>::hasConflicts() const
{
    assert(kRobust > 0); // TODO: consider kDelay==0 in the future (in which case, we also need to consider edge conflicts)
    ConstraintTable constraintTable;
    for (int i = 0; i < (int)al->paths_all.size() - 1; i++)
    {
        for (int t = 0; t < (int)al->paths_all[i].size(); t++) {
            if (al->paths_all[i][t].location == -1)
                continue;
            constraintTable.insert(al->paths_all[i][t].location, max(0, t - kRobust), t + kRobust + 1);
            if(t == al->paths_all[i].size() -1 && al->paths_all[i][t].location != (al->getAgent(i).goal_location.first * ml->cols + al->getAgent(i).goal_location.second)){
                cout<<"Agent: " << i << " didn't reach goal location"<<endl;
                return true;
            }
        }
        for (int j = i + 1; j < (int)al->paths_all.size(); j++)
        {
            for (int t = 0; t < (int)al->paths_all[j].size(); t++) {
                if (constraintTable.is_constrained(al->agents_all[j].agent_id, al->paths_all[j][t].location, t)) {
                		cout<<"Agent: "<<i <<","<<j <<" have conflict"<<endl;
                		cout<<"at t: "<<t<<" location:" << al->paths_all[j][t].location<<endl;
					return true;
				}
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
    result["best_initial_priority_strategy"] = this->bset_initisl_priority_strategy;


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
bool PythonCBS<Map>::PrioritizedPlaning(AgentsLoader* al, int thread_id, int priority_strategy)
{
    if (al == NULL)
        al = this->al;
    if (priority_strategy == -1)
        priority_strategy = this->agent_priority_strategy;
    if (options1.debug)
        cout << "Prioritized planning" << endl;
    int screen;
    screen = options1.debug;
    if (options1.debug)
        cout << "Sort the agents" << endl;
    al->generateAgentOrder(priority_strategy);


    auto runtime = (double)(time(NULL) - start_time);
    while (runtime < hard_time_limit) {
        cout << endl;
        al->updateToBePlannedAgents(1);
        if (al->num_of_agents == 0) // all agents have paths
            break;
        if (options1.debug)
        cout << "Remaining agents = " << al->getNumOfUnplannedAgents() <<
             ", remaining time = " << hard_time_limit - runtime << " seconds. " << endl
            << "Agent " << al->agents[0]->agent_id << endl;
        MultiMapICBSSearch <Map> icbs(ml, al, f_w, s, 0, screen, options1);
        icbs.ignoreFinishedAgent = true;
        bool res = icbs.runICBSSearch();
        updateCBSResults(icbs);
        if(!al->addPaths(icbs.paths))
        {
            cout << "The solution so far has conflicts!" << endl;
            hasConflicts(); // to print the conflict
            return false;
        }
        runtime = (double)(time(NULL) - start_time);
        int old_runtime = 0;
        if (!iteration_stats[thread_id].empty())
            old_runtime = get<2>(iteration_stats[thread_id].back());
        iteration_stats[thread_id].emplace_back(al->num_of_agents, 0,
                                     runtime, runtime - old_runtime,
                                     al->makespan,
                                     icbs.solution_cost,
                                     icbs.getSumOfHeuristicsAtStarts(),
                                     al->num_of_agents,
                                     al->getNumOfDeadAgents(),
                                     icbs.HL_num_expanded,
                                     icbs.LL_num_expanded);
    }

    runtime = (double)(time(NULL) - start_time);
    if (options1.debug)
    cout << endl << endl << "Find a solution for " << al->getNumOfAllAgents() - al->getNumOfUnplannedAgents()
         << " agents (including " << al->getNumOfDeadAgents() << " dead agents) in " << runtime << " seconds!" << endl;

    if (options1.debug)
    {
        al->printPaths();
    }
    return true;
}

template <class Map>
bool PythonCBS<Map>::GroupPrioritizedPlaning()
{
    int screen;
    if (options1.debug) {
        screen = 3;
    }
    else {
        screen = 0;
    }
    if (options1.debug)
        cout << "Sort the agents" << endl;
    al->generateAgentOrder(agent_priority_strategy);

    int groupSize = defaultGroupSize;
    runtime = (double)(time(NULL) - start_time);

    while (runtime < hard_time_limit) {
        if (options1.debug)
        cout << endl;
        al->updateToBePlannedAgents(groupSize);
        if (al->num_of_agents == 0) // all agents have paths
            break;
        runtime = (double)(time(NULL) - start_time);
        double time_limit = (hard_time_limit - runtime) * al->num_of_agents / al->getNumOfUnplannedAgents() / 2;
        if (options1.debug)
        cout << "Group size = " << al->num_of_agents <<
             ", time limit = " << time_limit << " seconds. " <<
             "(Remaining agents = " << al->getNumOfUnplannedAgents() <<
             ", remaining time = " << hard_time_limit - runtime << " seconds.) " << endl;
        if (options1.debug)
            cout << "initialize cbs search engine" << endl;

        if (options1.debug)
            cout << "Time limit = " << time_limit << "second." << endl;
        MultiMapICBSSearch <Map> icbs(ml, al, f_w, s, time_limit * CLOCKS_PER_SEC, screen, options1);
        icbs.corridor2 = corridor2;
        icbs.trainCorridor1 = trainCorridor1;
        icbs.ignoreFinishedAgent = true;
        icbs.chasing_reasoning = chasing;
        if (options1.debug)
            cout << "start search engine" << endl;
        bool res = icbs.runICBSSearch();
        updateCBSResults(icbs);
        int giveup_agents = 0;
        if (res) {
            groupSize = min(defaultGroupSize, al->num_of_agents * 2);
        }
        else
        {
            if (accept_partial_solution)
            {
                giveup_agents = icbs.getBestSolutionSoFar();
                if (options1.debug)
                cout << "Accept paths for " << al->num_of_agents - giveup_agents << " agents" << endl;
            }
            groupSize = max(1, al->num_of_agents / 2);
            if (options1.debug)
                cout << "Decreasing the group size to " << groupSize << endl;
        }
        if(!al->addPaths(icbs.paths))
        {
            cout << "The solution so far has conflicts!" << endl;
            hasConflicts(); // to print the conflict
            return false;
        }
        runtime = (double)(time(NULL) - start_time);
        int old_runtime = 0;
        if (!iteration_stats[0].empty())
            old_runtime = get<2>(iteration_stats[0].back());
        iteration_stats[0].emplace_back(al->num_of_agents, time_limit,
                                     runtime, runtime - old_runtime,
                                     al->makespan,
                                     icbs.solution_cost,
                                     icbs.getSumOfHeuristicsAtStarts(),
                                     al->num_of_agents - giveup_agents,
                                     al->getNumOfDeadAgents(),
                                     icbs.HL_num_expanded,
                                     icbs.LL_num_expanded);
    }

    runtime = (double)(time(NULL) - start_time);
    if (options1.debug)
    cout << endl << endl << "Find a solution for " << al->getNumOfAllAgents() - al->getNumOfUnplannedAgents()
         << " agents (including " << al->getNumOfDeadAgents() << " dead agents) in " << runtime << " seconds!" << endl;

    if (options1.debug)
    {
        al->printPaths();
    }
    return true;
}


template <class Map>
p::list PythonCBS<Map>::benchmarkSingleGroup(int group_size,int iterations, int time_limit) {
    p::list result;

    start_time = time(NULL);
    if (options1.debug)
        cout << "start initialize" << endl;
    //initialize search engine
    int screen;
    screen = options1.debug;
    al->constraintTable.init(ml->map_size());
    al->computeHeuristics(ml);
    if (options1.debug)
        cout << "Sort the agents" << endl;
    al->generateAgentOrder(agent_priority_strategy);

    int groupSize = group_size;
    runtime = (double)(time(NULL) - start_time);
    int num_iterations = iterations;
    while (iterations > 0) {
        al->sampleAgents(groupSize,iterations,num_iterations);
        if (al->num_of_agents == 0) // all agents have paths
            break;
        runtime = (double)(time(NULL) - start_time);
        cout << "Group size = " << al->num_of_agents <<
             ", time limit = " << time_limit << " seconds. " << endl;
        if (options1.debug)
            cout << "initialize cbs search engine" << endl;

        if (options1.debug)
            cout << "Time limit = " << time_limit << "second." << endl;
        MultiMapICBSSearch <Map> icbs(ml, al, f_w, s, time_limit * CLOCKS_PER_SEC, screen, options1);
        icbs.corridor2 = corridor2;
        icbs.trainCorridor1 = trainCorridor1;
        icbs.ignoreFinishedAgent = true;
        icbs.chasing_reasoning = chasing;
        if (options1.debug)
            cout << "start search engine" << endl;
        bool res = icbs.runICBSSearch();
        updateCBSResults(icbs);

        p::dict run;
        run["success"] = res;
        run["instance"] = iterations;
        run["runtime"] = icbs.runtime;
        run["group_size"] = group_size;
        run["corridor"] = corridor_option;
        run["HL_expanded"] = icbs.HL_num_expanded;
        run["HL_generated"] = icbs.HL_num_generated;
        run["LL_expanded"] = icbs.LL_num_expanded;
        run["LL_generated"] = icbs.LL_num_generated;
        std::stringstream stream;
        stream << std::fixed << std::setprecision(1) << f_w;
        std::string s = stream.str();
        run["algorithm"] = algo + "(" + s + ")_groupsize=" + to_string(defaultGroupSize) +
                              "_priority=" + to_string(agent_priority_strategy);
        run["num_standard"] = icbs.num_standard;
        run["num_chasing"] = icbs.num_chasing;
        run["num_start"] = icbs.num_start;
        run["num_corridor"] = icbs.num_corridor;
        run["runtime_corridor"] = icbs.runtime_corridor;
        size_t solution_cost = 0;
        int finished_agents = 0;
        size_t makespan = 0;
        for (const auto& path : al->paths_all)
        {
            solution_cost += path.size();
            makespan = max(path.size(), makespan);
            if (!path.empty())
                finished_agents++;
        }
        run["solution_cost"] = solution_cost;

        run["makespan"] = makespan;

        result.append(run);
        iterations --;
    }

    return result;
}

template <class Map>
p::list PythonCBS<Map>::benchmarkSingleGroupLNS(int group_size,int iterations, int time_limit) {
    p::list result;
    start_time = time(NULL);
    if (options1.debug)
        cout << "start initialize" << endl;
    //initialize search engine
    int screen=0;
    screen = options1.debug;
    al->constraintTable.init(ml->map_size());
    al->computeHeuristics(ml);
    if (options1.debug)
        cout << "Sort the agents" << endl;
    al->generateAgentOrder(agent_priority_strategy);

    PrioritizedPlaning();// get initial solution
    size_t solution_cost = 0;
    int finished_agents = 0;
    size_t makespan = 0;
    for (const auto& path : al->paths_all)
    {
        solution_cost += path.size();
        makespan = max(path.size(), makespan);
        if (!path.empty())
            finished_agents++;
    }
    runtime = (double)(time(NULL) - start_time);
    cout << "Solution cost = " << solution_cost << ", "
         << "makespan = " << makespan << ", "
         << "remaining time = " << hard_time_limit - runtime << endl;

    int groupSize = group_size;
    int num_iterations = iterations;
    boost::unordered_set<int> tabu_list;

    while (iterations > 0) {
        cout <<"Iteration: "<<iterations<<endl;
        // find the bottleneck agent
        int a = -1;
        for (int i = 0; i < al->paths_all.size(); i++)
        {
            if (tabu_list.find(i) != tabu_list.end())
                continue;
            if (a < 0 || al->paths_all[a].size() < al->paths_all[i].size())
            {
                a = i;
            }
        }
        tabu_list.insert(a);
        assert(a == al->agents_all[a].agent_id);
        al->constraintTable.delete_path(a, al->paths_all[a]);
        set<int> neighbors;
        int T = al->paths_all[a].size();
        int count = 0;
        while (neighbors.size() < groupSize - 1 && count < 10 && T > 0)
        {
            int t = ((iterations/num_iterations)*T) % T;
            generateNeighbor(a, al->paths_all[a][t], t, neighbors, groupSize - 1, (int) al->paths_all[a].size() - 1);
            T = t;
            count++;
        }
        if (neighbors.empty())
        {
            assert(a == al->agents_all[a].agent_id);
            al->constraintTable.insert_path(a, al->paths_all[a]);
            continue;
        }
        while (neighbors.size() < groupSize - 1)
        {
            int new_agent = ((iterations/num_iterations)*al->paths_all.size()+neighbors.size()) % al->paths_all.size();
            neighbors.insert(new_agent);
        }
        al->num_of_agents = (int)neighbors.size() + 1;
        al->agents.clear();
        al->agents.push_back(&al->agents_all[a]);
        cout << "Agents ids: " << a;
        int old_sum_of_costs = (int)al->paths_all[a].size() - 1;
        int old_makespan = (int)al->paths_all[a].size() - 1;
        for (auto i : neighbors)
        {
            old_sum_of_costs += (int)al->paths_all[i].size() - 1;
            old_makespan = max(old_makespan, (int)al->paths_all[i].size() - 1);
            assert(i == al->agents_all[i].agent_id);
            al->constraintTable.delete_path(i, al->paths_all[i]);
            al->agents.push_back(&al->agents_all[i]);
            cout << "," << i;
        }
        cout << endl;
        MultiMapICBSSearch <Map> icbs(ml, al, f_w, s, time_limit * CLOCKS_PER_SEC, screen, options1);
        icbs.corridor2 = corridor2;
        icbs.trainCorridor1 = trainCorridor1;
        icbs.ignoreFinishedAgent = true;
        icbs.chasing_reasoning = chasing;
        if (options1.debug)
            cout << "start search engine" << endl;
        bool res = icbs.runICBSSearch();
        updateCBSResults(icbs);


        p::dict run;
        run["success"] = res;
        run["instance"] = iterations;
        run["runtime"] = icbs.runtime;
        run["group_size"] = group_size;
        run["corridor"] = corridor_option;
        run["HL_expanded"] = icbs.HL_num_expanded;
        run["HL_generated"] = icbs.HL_num_generated;
        run["LL_expanded"] = icbs.LL_num_expanded;
        run["LL_generated"] = icbs.LL_num_generated;
        std::stringstream stream;
        stream << std::fixed << std::setprecision(1) << f_w;
        std::string s = stream.str();
        run["algorithm"] = algo + "(" + s + ")_groupsize=" + to_string(defaultGroupSize) +
                           "_priority=" + to_string(agent_priority_strategy);
        run["num_standard"] = icbs.num_standard;
        run["num_chasing"] = icbs.num_chasing;
        run["num_start"] = icbs.num_start;
        run["num_corridor"] = icbs.num_corridor;
        run["runtime_corridor"] = icbs.runtime_corridor;
        size_t solution_cost = 0;
        int finished_agents = 0;
        size_t makespan = 0;
        for (const auto& path : al->paths_all)
        {
            solution_cost += path.size();
            makespan = max(path.size(), makespan);
            if (!path.empty())
                finished_agents++;
        }
        run["solution_cost"] = solution_cost;

        run["makespan"] = makespan;

        result.append(run);
        assert(a == al->agents_all[a].agent_id);
        if(!al->constraintTable.insert_path(a, al->paths_all[a]))
            exit(11);
        for (auto i : neighbors)
        {
            assert(i == al->agents_all[i].agent_id);
            if(!al->constraintTable.insert_path(i, al->paths_all[i]))
                exit(13);
        }

        iterations --;
    }
    return result;

}

template <class Map>
void PythonCBS<Map>::generateNeighbor(int agent_id, const PathEntry& start, int start_time, set<int>& conflicting_agents, int neighbor_size, int upperbound)
{
    // a random walk with path that is shorter than upperbound and has conflicting with neighbor_size agents
    int speed = al->agents_all[agent_id].speed;
    const auto& heuristics = al->agents_all[agent_id].heuristics;
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
        runtime = (double)(time(NULL) - start_time);
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
    this->bset_initisl_priority_strategy = strategies[best_al];
    runtime = (double)(time(NULL) - start_time);
    if (options1.debug) {
        cout << "Best PP strategy = " << strategies[best_al] << ", "
             << "Final Solution cost = " << best_cost << ", "
             << "Final makespan = " << best_makespan << ", "
             << "Final runtime = " << runtime << endl;
    }
    return true;
}

template class PythonCBS<FlatlandLoader>;

template <class Map>
void PythonCBS<Map>::buildMCP(void)  // TODO: Ignore wait actions
{
    cout << "Start MCP ..." << endl;
    size_t map_size = ml->cols * ml->rows;
    cout << "map_size: " << map_size << endl;
    mcp.resize(map_size);
    agent_time.resize(al->getNumOfAllAgents(), -1);
    to_go.resize(al->getNumOfAllAgents(), -1);
    appear_time.resize(al->getNumOfAllAgents(), al->constraintTable.length_max + 1);
    size_t max_timestep = 0;
    for (int i = 0; i < al->getNumOfAllAgents(); i++)
        if (!al->paths_all[i].empty() && al->paths_all[i].size() > max_timestep)
            max_timestep = al->paths_all[i].size();
    cout << "max_timestep = " << max_timestep << endl;

    for (size_t t = 0; t < max_timestep; t++)
    {
        for (int i = 0; i < al->getNumOfAllAgents(); i++)
        {
            if (!al->paths_all[i].empty() && t < al->paths_all[i].size() && al->paths_all[i][t].location != -1)
            {
                mcp[al->paths_all[i][t].location].push_back(make_tuple(i, t));

                if (agent_time[i] == -1)
                    agent_time[i] = t;

                appear_time[i] = min(appear_time[i], (int)t);
                // cout << "Agent, time = " << get<0>(mcp[al->paths_all[i][t].location].back()) << ", "
                //     << get<1>(mcp[al->paths_all[i][t].location].back()) << endl;
            }
        }
    }
    cout << "End building MCP ..." << endl;
    return;
}

template<class Map>
p::list PythonCBS<Map>::getNextLoc(p::list agent_location, int timestep)
{
    for (int i = 0; i < al->getNumOfAllAgents(); i++)
    {
        // cout << "********************************************" << endl;
        // cout << "Agent: " << i << endl;
        // cout << "Next location: " << al->paths_all[i][agent_time[i]].location << endl;
        // cout << "MCP at location: " << get<0>(mcp[al->paths_all[i][agent_time[i]].location].front()) << ", " << 
        //     get<1>(mcp[al->paths_all[i][agent_time[i]].location].front()) << endl;
        // cout << "********************************************" << endl;

        if (!al->paths_all[i].empty() &&
            appear_time[i] <= timestep &&
            agent_time[i] < al->paths_all[i].size() &&
            !mcp[al->paths_all[i][agent_time[i]].location].empty())
        {
            int loc = al->paths_all[i][agent_time[i]].location;
            int first_agent = get<0>(mcp[loc].front());
            // int first_time = get<1>(mcp[loc].front());

            if (get<0>(mcp[loc].front()) == i || agent_time[i] == al->paths_all[i].size() - 1)
                to_go[i] = al->paths_all[i][agent_time[i]].location;
            
            else if (first_agent < i && mcp[loc].size() > 1)
            {
                if (get<0>(*std::next(mcp[loc].begin())) == i && // the second agent is i
                    agent_location[first_agent] == loc)  // the fist agent is already at loc
                    // agent_location[i] != al->paths_all[first_agent][agent_time[first_agent]].location) // not edge conflict
                {
                    to_go[i] = al->paths_all[i][agent_time[i]].location;
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

    boost::python::list next_loc;
    for (int i = 0; i < al->getNumOfAllAgents(); i++)
        next_loc.append(to_go[i]);

    return next_loc;
}

template<class Map>
void PythonCBS<Map>::updateMCP(p::list agent_location, p::dict agent_action)
{
    for (int i = 0; i < al->getNumOfAllAgents(); i++)
    {
        // Reach Goal
        if (agent_time[i] == al->paths_all[i].size() && 
            !mcp[al->paths_all[i][agent_time[i]-1].location].empty() && 
            get<0>(mcp[al->paths_all[i][agent_time[i]-1].location].front()) == i)
        {
            mcp[al->paths_all[i][agent_time[i]-1].location].pop_front();
        }

        else if (agent_location[i] != -1 && agent_location[i] == al->paths_all[i][agent_time[i]].location)
        {
            if (options1.debug)
            {
                cout << "agent " << i << endl;
                cout << "agent_time[i]: " << agent_time[i] << endl;
                cout << "MCP: a->" << get<0>(mcp[al->paths_all[i][agent_time[i]].location].front());
                cout << "  t->" << get<1>(mcp[al->paths_all[i][agent_time[i]].location].front()) <<endl;
            }
            // if (agent_action[i] != 4)
            //     assert(agent_time[i] == get<1>(mcp[al->paths_all[i][agent_time[i]].location].front()));

            if (agent_time[i] > 0 && 
                al->paths_all[i][agent_time[i]-1].location != -1 && 
                !mcp[al->paths_all[i][agent_time[i]-1].location].empty() && 
                get<0>(mcp[al->paths_all[i][agent_time[i]-1].location].front()) == i)
            {
                // cout << "Pop mcp: " << al->paths_all[i][agent_time[i]-1].location << endl;
                mcp[al->paths_all[i][agent_time[i]-1].location].pop_front();
            }

            if (agent_time[i] < al->paths_all[i].size())
                agent_time[i] ++;  // TODO: Change to next time step different from current location
        }
    }
    return;
}

template <class Map>
void PythonCBS<Map>::printAllMCP(void)
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
    return;
}

template <class Map>
void PythonCBS<Map>::printMCP(int loc)
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
    return;
}

template<class Map>
void PythonCBS<Map>::printAgentTime(void)
{
    cout << "==================== Time ====================" << endl;
    for (int i = 0; i < al->getNumOfAllAgents(); i++)
    {
        cout << "Agent " << i << ": " << agent_time[i] << endl;
    }
    cout << "================== End Time ==================" << endl;
}


BOOST_PYTHON_MODULE(libPythonCBS)  // Name here must match the name of the final shared library, i.e. mantid.dll or mantid.so
{
	using namespace boost::python;
	class_<PythonCBS<FlatlandLoader>>("PythonCBS", init<object, string, string, float,
	        int, int,float,int,bool,bool,int,int,int,int>())
		.def("getResult", &PythonCBS<FlatlandLoader>::getResult)
		.def("search", &PythonCBS<FlatlandLoader>::search)
		.def("benchmarkSingleGroup", &PythonCBS<FlatlandLoader>::benchmarkSingleGroup)
		.def("benchmarkSingleGroupLNS", &PythonCBS<FlatlandLoader>::benchmarkSingleGroupLNS)
		.def("hasConflicts", &PythonCBS<FlatlandLoader>::findConflicts)
		.def("getResultDetail", &PythonCBS<FlatlandLoader>::getResultDetail)
		.def("writeResultsToFile", &PythonCBS<FlatlandLoader>::writeResultsToFile)
		.def("updateAgents",&PythonCBS<FlatlandLoader>::updateAgents)
		.def("updateFw", &PythonCBS<FlatlandLoader>::updateFw)
        .def("buildMCP", &PythonCBS<FlatlandLoader>::buildMCP)
        .def("getNextLoc", &PythonCBS<FlatlandLoader>::getNextLoc)
        .def("updateMCP", &PythonCBS<FlatlandLoader>::updateMCP)
        .def("clearMCP", &PythonCBS<FlatlandLoader>::clearMCP)
        .def("printAllMCP", &PythonCBS<FlatlandLoader>::printAllMCP)
        .def("printMCP", &PythonCBS<FlatlandLoader>::printMCP)
        .def("printAgentTime", &PythonCBS<FlatlandLoader>::printAgentTime);
}