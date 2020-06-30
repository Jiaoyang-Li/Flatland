#include "PythonCBS.h"
#include "flat_map_loader.h"
#include "MDD.h"

namespace p = boost::python;




template <class Map>
PythonCBS<Map>::PythonCBS(p::object railEnv1, std::string algo, int kRobust, int t, bool debug, float f_w,string corridor) :railEnv(railEnv1) {
	//Initialize PythonCBS. Load map and agent info into memory
	std::cout << "algo: " << algo << std::endl;
	options1.debug = debug;
	options1.ignore_t0 = false;
	options1.shortBarrier = false;
	options1.asymmetry_constraint = false;
	timeLimit = t;
	this->f_w = f_w;
	this->algo = algo;
	this->kRobust = kRobust;
	if (corridor == "trainCorridor1") {
		this->trainCorridor1 = true;
		this->corridor2 = true;
	}
	if (corridor == "trainCorridor2") {
		this->trainCorridor2 = true;
		this->corridor2 = true;
	}
	if (corridor == "corridor2")
		this->corridor2 = true;
	if (corridor == "corridor4")
		this->corridor4 = true;
	if (algo == "ICBS")
		s = constraint_strategy::ICBS;
	else if (algo == "CBS")
		s = constraint_strategy::CBS;
	else if (algo == "CBSH")
		s = constraint_strategy::CBSH;
	else if (algo == "CBSH-CR")
		s = constraint_strategy::CBSH_CR;
	else if (algo == "CBSH-R")
		s = constraint_strategy::CBSH_R;
	else if (algo == "CBSH-RM")
		s = constraint_strategy::CBSH_RM;
	else if (algo == "CBSH-GR")
		s = constraint_strategy::CBSH_GR;
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

	if (options1.debug)
		cout << "Sort the agents" << endl;
	al->generateAgentOrder();

	int groupSize = defaultGroupSize;
	while (true) {
        al->updateToBePlannedAgents(groupSize);
        if (al->num_of_agents == 0) // all agents have paths
            break;
        runtime = (double)(std::clock() - start_time) / CLOCKS_PER_SEC;
        double time_limit = (timeLimit - runtime) * al->num_of_agents / al->getNumOfUnplannedAgents() / 4;
        cout << endl << "Group size = " << al->num_of_agents <<
                ", time limit = " << time_limit << " seconds." << endl;
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
        updateResults(icbs);
        if (res) {
            al->addPaths(icbs.paths);
            groupSize = defaultGroupSize;
        }
        else if (icbs.isTimeout()) // run out of time
        {
            runtime = (double)(std::clock() - start_time) / CLOCKS_PER_SEC;
            if (runtime  < timeLimit){
                groupSize = al->num_of_agents / 2;
                if (options1.debug)
                    cout << "Decreasing the group size to " << groupSize << endl;
            }
            else
                break;
        }
        else // no solutions, which should not happen
        {
            return false;
        }
    }

    runtime = (double)(std::clock() - start_time) / CLOCKS_PER_SEC;
    cout << endl << endl << "Find a solution for " << al->getNumOfAllAgents() - al->getNumOfUnplannedAgents()
            << " agents in " << runtime << " seconds!" << endl;

	if (options1.debug)
    {
	    if (hasConflicts(al->blocked_paths))
        {
            cout << "The final solution has conflicts!!!" << endl;
            return false;
        }
        for (int i = 0; i < (int)al->blocked_paths.size(); i++)
        {
            std::cout << "Agent " << i << ": ";
            for (int t = 0; t < (int)al->blocked_paths[i].size(); t++)
                std::cout << t <<"(" << al->blocked_paths[i][t].location << ")->";
            std::cout << std::endl;
        }
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
    for (const auto& path : al->blocked_paths)
    {
        solution_cost += path.size();
    }
    result["solution_cost"] = solution_cost;
	return result;

}


BOOST_PYTHON_MODULE(libPythonCBS)  // Name here must match the name of the final shared library, i.e. mantid.dll or mantid.so
{
	using namespace boost::python;
	class_<PythonCBS<FlatlandLoader>>("PythonCBS", init<object, string, int, int, bool,float,string>())
		.def("getResult", &PythonCBS<FlatlandLoader>::getResult)
		.def("search", &PythonCBS<FlatlandLoader>::search)
		.def("getResultDetail", &PythonCBS<FlatlandLoader>::getResultDetail)
		.def("updateAgents",&PythonCBS<FlatlandLoader>::updateAgents)
		.def("updateFw", &PythonCBS<FlatlandLoader>::updateFw);

}