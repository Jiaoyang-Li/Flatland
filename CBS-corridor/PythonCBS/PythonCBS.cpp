#include "PythonCBS.h"
#include "flat_map_loader.h"
#include "MDD.h"

namespace p = boost::python;




template <class Map>
PythonCBS<Map>::PythonCBS(p::object railEnv1, std::string algo, int kRobust, int t,
                          int default_group_size, bool debug, float f_w, string corridor, bool accept_partial_solution) :
                          railEnv(railEnv1), defaultGroupSize(default_group_size),
                          accept_partial_solution(accept_partial_solution) {
	//Initialize PythonCBS. Load map and agent info into memory
	std::cout << "algo: " << algo << std::endl;
	options1.debug = debug;
	timeLimit = t;
	this->f_w = f_w;
	this->algo = algo;
	this->kRobust = kRobust;
    if (corridor == "trainCorridor1") {
		this->trainCorridor1 = true;
		this->corridor2 = true;
	}
	if (corridor == "corridor2")
		this->corridor2 = true;
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
	int screen;
	if (options1.debug) {
		screen = 3;
	}
	else {
		screen = 0;
	}
    al->computeHeuristics(ml);
	if (options1.debug)
		cout << "Sort the agents" << endl;
	al->generateAgentOrder();

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
        MultiMapICBSSearch <Map> icbs(ml, al, f_w, s, time_limit * CLOCKS_PER_SEC, screen, kRobust, options1);
        if(s == constraint_strategy::CBSH_RM)
            icbs.rectangleMDD = true;
        icbs.trainCorridor1 = trainCorridor1;
        icbs.trainCorridor2 = trainCorridor2;
        icbs.corridor2 = corridor2;
        icbs.corridor4 = corridor4;
        icbs.ignoreFinishedAgent = true;
        icbs.max_malfunction = this->max_malfunction;
        if (options1.debug)
            cout << "start search engine" << endl;
        bool res = icbs.runICBSSearch();
        updateCBSResults(icbs);
        if (res) {
            groupSize = min(defaultGroupSize, al->num_of_agents * 2);
        }
        else
        {
            if (accept_partial_solution)
            {
                int giveup_agents = icbs.getBestSolutionSoFar();
                cout << "Accept paths for " << al->num_of_agents - giveup_agents << " agents" << endl;
            }
            groupSize = max(1, al->num_of_agents / 2);
            if (options1.debug)
                cout << "Decreasing the group size to " << groupSize << endl;
        }
        al->addPaths(icbs.paths, kRobust);
        if (options1.debug && hasConflicts(al->paths_all))
            return false; // The solution has conflicts! There should be some bugs in the code
        runtime = (double)(std::clock() - start_time) / CLOCKS_PER_SEC;
        iteration_stats.emplace_back(al->num_of_agents, time_limit,
                                     runtime, icbs.solution_cost,
                                     icbs.getSumOfHeuristicsAtStarts(),
                                     icbs.HL_num_expanded, icbs.LL_num_expanded);
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
            if(t == paths[i].size() -1 && paths[i][t].location != (al->agents[i]->goal_location.first * ml->cols + al->agents[i]->goal_location.second)){
                cout<<"Agent: " << i << " didn't reach goal location"<<endl;
                return false;
            }
        }
        for (int j = i + 1; j < (int)paths.size(); j++)
        {
            for (int t = 0; t < (int)paths[j].size(); t++) {
                if (constraintTable.is_constrained(paths[j][t].location, t)) {
                	if(options1.debug){
                		cout<<"Agent: "<<i <<","<<j<<endl;
                		cout<<"t: "<<t<<" location:" << paths[j][t].location<<endl;
                	}
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
	result["algorithm"] = algo;
	result["No_f_rectangle"] = num_rectangle;
	result["num_corridor2"] = num_corridor2;
	result["num_corridor4"] = num_corridor4;
    size_t solution_cost = 0;
    for (const auto& path : al->paths_all)
    {
        solution_cost += path.size();
    }
    result["solution_cost"] = solution_cost;
	return result;
}


BOOST_PYTHON_MODULE(libPythonCBS)  // Name here must match the name of the final shared library, i.e. mantid.dll or mantid.so
{
	using namespace boost::python;
	class_<PythonCBS<FlatlandLoader>>("PythonCBS", init<object, string, int, int, int, bool,float,string,bool>())
		.def("getResult", &PythonCBS<FlatlandLoader>::getResult)
		.def("search", &PythonCBS<FlatlandLoader>::search)
		.def("getResultDetail", &PythonCBS<FlatlandLoader>::getResultDetail)
		.def("writeResultsToFile", &PythonCBS<FlatlandLoader>::writeResultsToFile)
		.def("updateAgents",&PythonCBS<FlatlandLoader>::updateAgents)
		.def("updateFw", &PythonCBS<FlatlandLoader>::updateFw);

}