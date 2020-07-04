/* Copyright (C) Jiaoyang Li
* Unauthorized copying of this file, via any medium is strictly prohibited
* Confidential
* Written by Jiaoyang Li <jiaoyanl@usc.edu>, Dec 2018
*/

/*driver.cpp
* Solve a MAPF instance on 2D grids.
*/

#include "map_loader.h"
#include "agents_loader.h"
#include "ICBSSearch.h"

#include <iostream>
#include <fstream>

#include <boost/program_options.hpp>
#include<boost/tokenizer.hpp>

int main(int argc, char** argv) 
{
	namespace po = boost::program_options;
	// Declare the supported options.
	po::options_description desc("Allowed options");
	desc.add_options()
		("help", "produce help message")
		("map,m", po::value<std::string>()->required(), "input file for map")
		("agents,a", po::value<std::string>()->required(), "input file for agents")
		("output,o", po::value<std::string>()->required(), "output file for schedule")
		("solver,s", po::value<std::string>()->required(), "solvers (CBS, ICBS, CBSH, CBSH-CR, CBSH-R, CBSH-RM, CBSH-GR")
		("agentNum,k", po::value<int>()->default_value(0), "number of agents")
		("cutoffTime,t", po::value<float>()->default_value(7200), "cutoff time (seconds)")
		("seed,d", po::value<int>()->default_value(0), "random seed")
		("screen", po::value<int>()->default_value(0), "screen option (0: none; 1: results; 2:all)")
		("cardinalRect", po::value<bool>(), "only consider cardinal rectangle conflicts")
		("corridor2", po::value<bool>(), "reason about 2-way branching corridor conflicts")
		("corridor4", po::value<bool>(), "reason about 4-way branching corridor conflicts")
		("cardinalCorridor", po::value<bool>(), "only reason about cardinal corridor conflicts")
		("kDelay", po::value<int>()->default_value(0), "generate k-robust plan")
		("only_generate_instance", po::value<std::string>()->default_value(""),"no searching")
		("debug", "debug mode")
		("flipped_rectangle", "resolving flipped rectangle symmetry conflict for RM")

	;

	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);

	if (vm.count("help")) {
		std::cout << desc << std::endl;
		return 1;
	}

	po::notify(vm);
	srand((int)time(0));

	if (vm["screen"].as<int>() == 2) {
		cout << "[CBS] Loading map and agents " << endl;
	}
	// read the map file and construct its two-dim array
	MapLoader* ml = new MapLoader(vm["map"].as<string>());

	// read agents' start and goal locations
	AgentsLoader* al = new AgentsLoader(vm["agents"].as<string>(), *ml, vm["agentNum"].as<int>());
	if (vm["screen"].as<int>() == 2) {
		cout << "[CBS] Loading map and agents don2! " << endl;
	}
	srand(vm["seed"].as<int>());

	options options1;


	if (vm.count("debug")) {
		options1.debug = true;
	}
	else {
		options1.debug = false;
	}

	if (vm["only_generate_instance"].as<string>()!="") {
		al->saveToFile(vm["only_generate_instance"].as<string>());
		return 0;
	}

	constraint_strategy s;
	if (vm["solver"].as<string>() == "ICBS")
		s = constraint_strategy::ICBS;
	else if (vm["solver"].as<string>() == "CBS")
		s = constraint_strategy::CBS;
	else if (vm["solver"].as<string>() == "CBSH")
		s = constraint_strategy::CBSH;
	else
	{
		std::cout <<"WRONG SOLVER NAME!" << std::endl;
		return -1;
	}

	al->generateAgentOrder(0);
	al->updateToBePlannedAgents();
	MultiMapICBSSearch<MapLoader> icbs(ml, al, 1.0, s, vm["cutoffTime"].as<float>() * CLOCKS_PER_SEC, vm["screen"].as<int>(), vm["kDelay"].as<int>(), options1);

	if (vm.count("cardinalRect"))
	{
		icbs.cardinalRect = vm["cardinalRect"].as<bool>();
	}
	if (vm.count("corridor2"))
	{
		icbs.corridor2 = vm["corridor2"].as<bool>();
	}
	if (vm.count("corridor4"))
	{
		icbs.corridor4 = vm["corridor4"].as<bool>();
	}
	if (vm.count("cardinalCorridor"))
	{
		icbs.cardinalCorridorReasoning = vm["cardinalCorridor"].as<bool>();
	}
	
	bool res;
	res = icbs.runICBSSearch();
	ofstream stats;
	stats.open(vm["output"].as<string>(), ios::app);
	stats << icbs.runtime/ CLOCKS_PER_SEC << "," <<
		icbs.HL_num_expanded << "," << icbs.HL_num_generated << "," <<
		icbs.LL_num_expanded << "," << icbs.LL_num_generated << "," <<
		vm["agents"].as<string>() << "," << icbs.solution_cost << "," << 
		get<2>(icbs.min_f_val) - icbs.dummy_start->g_val << "," <<
		vm["solver"].as<string>()  << "," <<
		icbs.num_standard << "," << icbs.num_rectangle << "," <<
		icbs.num_corridor2 << "," << icbs.num_corridor4 << "," << 
		icbs.num_target<<","<< icbs.num_0FlipRectangle<<","<<
		icbs.num_1FlipRectangle << ","<< icbs.num_2FlipRectangle << endl;
	stats.close();
	if (vm["screen"].as<int>() == 2)
		cout << "Done!!" << endl;
	delete ml;
	return 0;

}
