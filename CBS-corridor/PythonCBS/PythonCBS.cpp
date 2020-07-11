#include "PythonCBS.h"
#include "flat_map_loader.h"
#include "MDD.h"
#include <iomanip>
#include <sstream>
#include <math.h>       /* log2 */
namespace p = boost::python;




template <class Map>
PythonCBS<Map>::PythonCBS(p::object railEnv1, string framework, std::string algo, int t,
                          int default_group_size, bool debug, float f_w, int corridor,bool chasing, bool accept_partial_solution,
                          int agent_priority_strategy) :
                          railEnv(railEnv1), framework(framework), defaultGroupSize(default_group_size),
                          accept_partial_solution(accept_partial_solution),
                          agent_priority_strategy(agent_priority_strategy) {
	//Initialize PythonCBS. Load map and agent info into memory
	std::cout << "algo: " << algo << std::endl;
	options1.debug = debug;
	timeLimit = t;
    srand(0);
    this->f_w = f_w;
	this->algo = algo;
	this->kRobust = 1;
	this->chasing = chasing;
	if(corridor > 0){
	    this->corridor2 = true;
	}
	if(corridor = 1){
	    this->trainCorridor1 = true;
	}
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

	std::cout << "get width height " << std::endl;
	p::long_ rows(railEnv.attr("height"));
	p::long_ cols(railEnv.attr("width"));

	std::cout << "load map " << p::extract<int>(rows)<<" x "<< p::extract<int>(cols) << std::endl;
	//ml =  new MapLoader(railEnv.attr("rail"), p::extract<int>(rows), p::extract<int>(cols));
	ml = new FlatlandLoader(railEnv.attr("rail"), p::extract<int>(rows), p::extract<int>(cols));
	std::cout << "load agents " << std::endl;

	al =  new AgentsLoader(railEnv.attr("agents"));
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
    start_time = std::clock();
    if (options1.debug)
		cout << "start initialize" << endl;
	//initialize search engine
	al->constraintTable.init(ml->map_size());
    al->computeHeuristics(ml);
    if (framework == "LNS")
        return LNS();
    else
	    return GroupPrioritizedPlaning();
}

template <class Map>
bool PythonCBS<Map>::hasConflicts(const vector<Path>& paths) const
{
    assert(kRobust > 0); // TODO: consider kDelay==0 in the future (in which case, we also need to consider edge conflicts)
    ConstraintTable constraintTable;
    for (int i = 0; i < (int)paths.size() - 1; i++)
    {
        for (int t = 0; t < (int)paths[i].size(); t++) {
            if (paths[i][t].location == -1)
                continue;
            constraintTable.insert(paths[i][t].location, max(0, t - kRobust), t + kRobust + 1);
            if(t == paths[i].size() -1 && paths[i][t].location != (al->getAgent(i).goal_location.first * ml->cols + al->getAgent(i).goal_location.second)){
                cout<<"Agent: " << i << " didn't reach goal location"<<endl;
                return true;
            }
        }
        for (int j = i + 1; j < (int)paths.size(); j++)
        {
            for (int t = 0; t < (int)paths[j].size(); t++) {
                if (constraintTable.is_constrained(paths[j][t].location, t)) {
                		cout<<"Agent: "<<i <<","<<j <<" have conflict"<<endl;
                		cout<<"at t: "<<t<<" location:" << paths[j][t].location<<endl;
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

	result["runtime"] = runtime;
	result["HL_expanded"] = HL_num_expanded;
	result["HL_generated"] = HL_num_generated;
	result["LL_expanded"] = LL_num_expanded;
	result["LL_generated"] = LL_num_generated;
    std::stringstream stream;
    stream << std::fixed << std::setprecision(1) << f_w;
    std::string s = stream.str();
	result["algorithm"] = algo + "(" + s + ")_groupsize=" + to_string(defaultGroupSize) +
	        "_priority=" + to_string(agent_priority_strategy);
	result["No_f_rectangle"] = num_rectangle;
	result["num_chasing"] = num_chasing;
	result["num_corridor2"] = num_corridor2;
    result["num_start"] = num_start;
    result["num_semi_corridor"] = num_semi_corridor;
	result["runtime_corridor"] = runtime_corridor;

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
    result["solution_cost"] = solution_cost;
    result["finished_agents"] = finished_agents;
    result["makespan"] = makespan;
	return result;
}

template <class Map>
bool PythonCBS<Map>::PrioritizedPlaning()
{
    if (options1.debug)
        cout << "Prioritized planning" << endl;
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


    runtime = (double)(std::clock() - start_time) / CLOCKS_PER_SEC;
    while (runtime < timeLimit) {
        cout << endl;
        al->updateToBePlannedAgents(1);
        if (al->num_of_agents == 0) // all agents have paths
            break;
        cout << "Remaining agents = " << al->getNumOfUnplannedAgents() <<
             ", remaining time = " << timeLimit - runtime << " seconds. " << endl;

        MultiMapICBSSearch <Map> icbs(ml, al, f_w, s, 0, screen, options1);
        icbs.ignoreFinishedAgent = true;
        bool res = icbs.runICBSSearch();
        updateCBSResults(icbs);
        if(!al->addPaths(icbs.paths, kRobust))
        {
            cout << "The solution so far has conflicts!" << endl;
            hasConflicts(al->paths_all); // to print the conflict
            return false;
        }
        runtime = (double)(std::clock() - start_time) / CLOCKS_PER_SEC;
        int old_runtime = 0;
        if (!iteration_stats.empty())
            old_runtime = get<2>(iteration_stats.back());
        iteration_stats.emplace_back(al->num_of_agents, 0,
                                     runtime, runtime - old_runtime,
                                     al->makespan,
                                     icbs.solution_cost,
                                     icbs.getSumOfHeuristicsAtStarts(),
                                     al->num_of_agents,
                                     al->getNumOfDeadAgents(),
                                     icbs.HL_num_expanded,
                                     icbs.LL_num_expanded);
    }

    runtime = (double)(std::clock() - start_time) / CLOCKS_PER_SEC;
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
    runtime = (double)(std::clock() - start_time) / CLOCKS_PER_SEC;

    while (runtime < timeLimit) {
        cout << endl;
        al->updateToBePlannedAgents(groupSize);
        if (al->num_of_agents == 0) // all agents have paths
            break;
        runtime = (double)(std::clock() - start_time) / CLOCKS_PER_SEC;
        double time_limit = (timeLimit - runtime) * al->num_of_agents / al->getNumOfUnplannedAgents() / 2;
        cout << "Group size = " << al->num_of_agents <<
             ", time limit = " << time_limit << " seconds. " <<
             "(Remaining agents = " << al->getNumOfUnplannedAgents() <<
             ", remaining time = " << timeLimit - runtime << " seconds.) " << endl;
        if (options1.debug)
            cout << "initialize cbs search engine" << endl;

        if (options1.debug)
            cout << "Time limit = " << time_limit << "second." << endl;
        MultiMapICBSSearch <Map> icbs(ml, al, f_w, s, time_limit * CLOCKS_PER_SEC, screen, options1);
        icbs.trainCorridor1 = trainCorridor1;
        icbs.corridor2 = corridor2;
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
                cout << "Accept paths for " << al->num_of_agents - giveup_agents << " agents" << endl;
            }
            groupSize = max(1, al->num_of_agents / 2);
            if (options1.debug)
                cout << "Decreasing the group size to " << groupSize << endl;
        }
        if(!al->addPaths(icbs.paths, kRobust))
        {
            cout << "The solution so far has conflicts!" << endl;
            hasConflicts(al->paths_all); // to print the conflict
            return false;
        }
        runtime = (double)(std::clock() - start_time) / CLOCKS_PER_SEC;
        int old_runtime = 0;
        if (!iteration_stats.empty())
            old_runtime = get<2>(iteration_stats.back());
        iteration_stats.emplace_back(al->num_of_agents, time_limit,
                                     runtime, runtime - old_runtime,
                                     al->makespan,
                                     icbs.solution_cost,
                                     icbs.getSumOfHeuristicsAtStarts(),
                                     al->num_of_agents - giveup_agents,
                                     al->getNumOfDeadAgents(),
                                     icbs.HL_num_expanded,
                                     icbs.LL_num_expanded);
    }

    runtime = (double)(std::clock() - start_time) / CLOCKS_PER_SEC;
    cout << endl << endl << "Find a solution for " << al->getNumOfAllAgents() - al->getNumOfUnplannedAgents()
         << " agents (including " << al->getNumOfDeadAgents() << " dead agents) in " << runtime << " seconds!" << endl;

    if (options1.debug)
    {
        al->printPaths();
    }
    return true;
}


template <class Map>
bool PythonCBS<Map>::LNS()
{
    int screen;
    if (options1.debug) {
        screen = 3;
    }
    else {
        screen = 0;
    }
    if (!PrioritizedPlaning()) // get initial solution
        return false;

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
    runtime = (double)(std::clock() - start_time) / CLOCKS_PER_SEC;
    cout << "Solution cost = " << solution_cost << ", "
         << "makespan = " << makespan << ", "
         << "remaining time = " << timeLimit - runtime << endl;

    boost::unordered_set<int> tabu_list;
    while (runtime < timeLimit)
    {
        if (tabu_list.size() >= al->paths_all.size() / 2)
            tabu_list.clear();
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
        al->constraintTable.delete_path(a, al->paths_all[a]);
        set<int> neighbors;
        int T = al->paths_all[a].size();
        int count = 0;
        while (neighbors.size() < defaultGroupSize - 1 && count < 10 && T > 0)
        {
            int t = rand() % T;
            generateNeighbor(a, al->paths_all[a][t], t, neighbors, defaultGroupSize - 1, (int)al->paths_all[a].size() - 1);
            T = t;
            count++;
        }
        if (neighbors.empty())
        {
            al->constraintTable.insert_path(a, al->paths_all[a]);
            continue;
        }
        al->num_of_agents = (int)neighbors.size() + 1;
        al->agents.clear();
        al->agents.push_back(&al->agents_all[a]);
        cout << "Agents ids: " << a << "(" << al->paths_all[a].size() - 1 << ")";
        for (auto i : neighbors)
        {
            al->constraintTable.delete_path(i, al->paths_all[i]);
            al->agents.push_back(&al->agents_all[i]);
            cout << "," << i << "(" << al->paths_all[i].size() - 1 << ")";
        }
        cout << endl;
        runtime = (double)(std::clock() - start_time) / CLOCKS_PER_SEC;
        if (runtime >= timeLimit)
            break;
        double time_limit = min(timeLimit - runtime, 10.0);
        MultiMapICBSSearch <Map> icbs(ml, al, f_w, s, time_limit * CLOCKS_PER_SEC, screen, options1);
        icbs.trainCorridor1 = trainCorridor1;
        icbs.corridor2 = corridor2;
        icbs.ignoreFinishedAgent = true;
        icbs.chasing_reasoning = chasing;
        if (options1.debug)
            cout << "start search engine" << endl;
        bool res = icbs.runICBSSearch();
        updateCBSResults(icbs);
        if (res)
        {
            assert(icbs.paths[0]->back().location == al->paths_all[a].back().location);
            if(!al->constraintTable.insert_path(a, *icbs.paths[0]))
                exit(10);
            //cout << a << ": " << al->paths_all[a].size() - 1 << "->" << icbs.paths[0]->size() - 1 << endl;
            al->paths_all[a] = *icbs.paths[0];
            int i = 1;
            for (auto n : neighbors)
            {
                assert(icbs.paths[i]->back().location == al->paths_all[n].back().location);
                if(!al->constraintTable.insert_path(n, *icbs.paths[i]))
                    exit(10);
                //cout << n << ": " << al->paths_all[n].size() - 1 << "->" << icbs.paths[i]->size() - 1 << endl;
                al->paths_all[n] = *icbs.paths[i];
                i++;
            }
        }
        else
        {
            if(!al->constraintTable.insert_path(a, al->paths_all[a]))
                exit(11);
            for (auto i : neighbors)
            {
                if(!al->constraintTable.insert_path(i, al->paths_all[i]))
                    exit(13);
            }
        }
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
        runtime = (double)(std::clock() - start_time) / CLOCKS_PER_SEC;
        cout << "Solution cost = " << solution_cost << ", "
             << "makespan = " << makespan  << ", "
             << "remaining time = " << timeLimit - runtime << endl;

        int old_runtime = 0;
        if (!iteration_stats.empty())
            old_runtime = get<2>(iteration_stats.back());
        iteration_stats.emplace_back(al->num_of_agents, time_limit,
                                     runtime, runtime - old_runtime,
                                     makespan,
                                     solution_cost,
                                     0,
                                     al->num_of_agents,
                                     al->agents_all.size() - finished_agents,
                                     icbs.HL_num_expanded,
                                     icbs.LL_num_expanded);
    }
    return true;
}

template <class Map>
void PythonCBS<Map>::generateNeighbor(int agent_id, const PathEntry& start, int start_time,  set<int>& conflicting_agents, int neighbor_size, int upperbound)
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
                al->constraintTable.get_conflicting_agents(conflicting_agents, loc, t + 1);
                break;
            }
            transitions.erase(it);
        }
        if (transitions.empty() || conflicting_agents.size() >= neighbor_size || h_val == 0)
            break;
    }
}

BOOST_PYTHON_MODULE(libPythonCBS)  // Name here must match the name of the final shared library, i.e. mantid.dll or mantid.so
{
	using namespace boost::python;
	class_<PythonCBS<FlatlandLoader>>("PythonCBS", init<object, string, string, int, int, bool,float,int,bool,bool,int>())
		.def("getResult", &PythonCBS<FlatlandLoader>::getResult)
		.def("search", &PythonCBS<FlatlandLoader>::search)
		.def("getResultDetail", &PythonCBS<FlatlandLoader>::getResultDetail)
		.def("writeResultsToFile", &PythonCBS<FlatlandLoader>::writeResultsToFile)
		.def("updateAgents",&PythonCBS<FlatlandLoader>::updateAgents)
		.def("updateFw", &PythonCBS<FlatlandLoader>::updateFw);

}