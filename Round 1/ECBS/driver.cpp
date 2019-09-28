#include "grid2D.h"
#include "ecbs_search.h"
#include "ecbs_search.cpp"

#include "boost/program_options.hpp"
#include <boost/property_tree/ptree.hpp>

namespace pt = boost::property_tree;

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
		("weight,w", po::value<double>()->required(), "suboptimal bound for ECBS")
		("disjoint,d", po::value<bool>()->default_value(false), "disjoint splitting")		
		("screen,s", po::value<int>()->default_value(0), "screen (0: only results; 1: details)")
		("cutoffTime,t", po::value<int>()->default_value(300), "cutoff time (seconds)")
		("seed", po::value<int>()->default_value(0), "random seed")
	;

	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);

	if (vm.count("help")) {
		std::cout << desc << std::endl;
		return 1;
	}

	po::notify(vm);

	Grid2D G;
	G.load_map(vm["map"].as<string>());
	G.load_agents(vm["agents"].as<string>());
	G.preprocessing_heuristics();

	srand(vm["seed"].as<int>());

	ECBSSearch<Grid2D> ecbs(G, vm["weight"].as<double>(), vm["disjoint"].as<bool>(), vm["cutoffTime"].as<int>());
	ecbs.screen = vm["screen"].as<int>();
	ecbs.runECBSSearch();
	ecbs.saveResults(vm["output"].as<string>(), vm["agents"].as<string>());

	return 0;

}
