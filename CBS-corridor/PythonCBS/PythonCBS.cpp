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
		al->printAgentsInitGoal();
	}
	this->max_malfunction = max_malfunction;

}


template <class Map>
void PythonCBS<Map>::updateAgents(p::object railEnv1) {
	if (options1.debug)
		cout << "update Agents" << endl;
	al->updateAgents(railEnv.attr("agents"));
	
	if (icbs != NULL)
		delete icbs;
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

	return icbs->outputPaths();
}

template <class Map>
bool PythonCBS<Map>::search() {
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
		cout << "initialize cbs search engine" << endl;
	icbs = new MultiMapICBSSearch <Map> (ml, al, f_w, s, timeLimit * CLOCKS_PER_SEC,screen, kRobust, options1);
	if(s == constraint_strategy::CBSH_RM)
		icbs->rectangleMDD = true;
	icbs->trainCorridor1 = trainCorridor1;
	icbs->trainCorridor2 = trainCorridor2;
	icbs->corridor2 = corridor2;
	icbs->corridor4 = corridor4;
	icbs->ignoreFinishedAgent = true;
	icbs->max_malfunction = this->max_malfunction;
	bool res =false;
	if (options1.debug)
		cout << "start search engine" << endl;
	res = icbs->runICBSSearch();

	
	return res;

}


template <class Map>
p::dict PythonCBS<Map>::getResultDetail() {
	//return result detail
	p::dict result;

	result["runtime"] = icbs->runtime / CLOCKS_PER_SEC;
	result["HL_expanded"] = icbs->HL_num_expanded;
	result["HL_generated"] = icbs->HL_num_generated;

	result["LL_expanded"] = icbs->LL_num_expanded;
	result["LL_generated"] = icbs->LL_num_generated;
	if (icbs->isTimeout())
		result["solution_cost"] = -1;
	else
		result["solution_cost"] = icbs->solution_cost;
	result["algorithm"] = algo;
	result["No_f_rectangle"] = icbs->num_rectangle;
	result["num_corridor2"] = icbs->num_corridor2;
	result["num_corridor4"] = icbs->num_corridor4;
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