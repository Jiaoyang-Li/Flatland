#include "PythonCBS.h"
#include "flat_map_loader.h"
#include "MDD.h"
#include <iomanip>
#include <sstream>
#include <math.h>       /* log2 */
namespace p = boost::python;




template <class Map>
PythonCBS<Map>::PythonCBS(p::object railEnv1, std::string algo, int kRobust, int t,
                          int default_group_size, bool debug, float f_w, int corridor,bool chasing, bool accept_partial_solution,
                          int agent_priority_strategy) :
                          railEnv(railEnv1), defaultGroupSize(default_group_size),
                          accept_partial_solution(accept_partial_solution),
                          agent_priority_strategy(agent_priority_strategy) {
	//Initialize PythonCBS. Load map and agent info into memory
	std::cout << "algo: " << algo << std::endl;
	options1.debug = debug;
	timeLimit = t;
    srand(0);
    this->f_w = f_w;
	this->algo = algo;
	this->kRobust = kRobust;
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
	al->generateAgentOrder(agent_priority_strategy);

	int groupSize = defaultGroupSize;
    runtime = (double)(std::clock() - start_time) / CLOCKS_PER_SEC;

	while (runtime < timeLimit) {
        cout << endl;
        al->updateToBePlannedAgents(groupSize);
        if (al->num_of_agents == 0) // all agents have paths
            break;
        runtime = (double)(std::clock() - start_time) / CLOCKS_PER_SEC;
        double time_limit = (timeLimit - runtime) * al->num_of_agents / al->getNumOfUnplannedAgents() / (1+0.5*log2(al->num_of_agents));
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
                                     al->constraintTable.latest_timestep,
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
void PythonCBS<Map>::buildMCP(void)
{
    int map_size = ml->cols * ml->rows;
    mcp.resize(map_size);
    agent_time.resize(al->getNumOfAllAgents(), 0);
    to_go.resize(al->getNumOfAllAgents(), -1);

    size_t max_timestep = 0;
    for (int i = 0; i < al->getNumOfAllAgents(); i++)
        if (!al->paths_all[i].empty() && al->paths_all[i].size() > max_timestep)
            max_timestep = al->paths_all[i].size();

    for (size_t t = 0; t < max_timestep; t++)
    {
        for (int i = 0; i < al->getNumOfAllAgents(); i++)
        {
            if (!al->paths_all[i].empty() && al->paths_all[i][t].location != -1)
            {
                mcp[al->paths_all[i][t].location].emplace_back(i, t);
            }
        }
    }
    return;
}

template<class Map>
p::list PythonCBS<Map>::getNextLoc(void)
{
    for (int i = 0; i < al->getNumOfAllAgents(); i++)
    {
        if (!al->paths_all[i].empty() && get<0>(mcp[al->paths_all[i][agent_time[i]].location].front()) == i)
        {
            to_go[i] = al->paths_all[i][agent_time[i]].location;
        }
    }

    boost::python::list next_loc;
    for (int i = 0; i < al->getNumOfAllAgents(); i++)  
        next_loc.append(to_go[i]);

    return next_loc;
}

template<class Map>
void PythonCBS<Map>::updateMCP(p::list agent_location)
{
    for (int i = 0; i < al->getNumOfAllAgents(); i++)
    {
        if (agent_location[i] == to_go[i])
        {
            mcp[al->paths_all[i][agent_time[i]].location].pop_front();
            agent_time[i] ++;
        }
    }
    return;
}

template <class Map>
void PythonCBS<Map>::printMCP(void)
{
    cout << "==================== MCP ====================" << endl;
    for (const auto& m: mcp)
    {
        auto &last = *(--m.end());
        for (const auto& p: m)
        {
            cout << "(" << get<0>(p) << "," << get<1>(p) << ")";
            if (&p != &last)
                cout << "->";
        }
    }
    cout << "================== MCP END ==================" << endl;
    return;
}


BOOST_PYTHON_MODULE(libPythonCBS)  // Name here must match the name of the final shared library, i.e. mantid.dll or mantid.so
{
	using namespace boost::python;
	class_<PythonCBS<FlatlandLoader>>("PythonCBS", init<object, string, int, int, int, bool,float,int,bool,bool,int>())
		.def("getResult", &PythonCBS<FlatlandLoader>::getResult)
		.def("search", &PythonCBS<FlatlandLoader>::search)
		.def("getResultDetail", &PythonCBS<FlatlandLoader>::getResultDetail)
		.def("writeResultsToFile", &PythonCBS<FlatlandLoader>::writeResultsToFile)
		.def("updateAgents",&PythonCBS<FlatlandLoader>::updateAgents)
		.def("updateFw", &PythonCBS<FlatlandLoader>::updateFw)
        .def("buildMCP", &PythonCBS<FlatlandLoader>::buildMCP)
        .def("getNextLoc", &PythonCBS<FlatlandLoader>::getNextLoc)
        .def("printMCP", &PythonCBS<FlatlandLoader>::printMCP)
        .def("updateMCP", &PythonCBS<FlatlandLoader>::updateMCP);
}