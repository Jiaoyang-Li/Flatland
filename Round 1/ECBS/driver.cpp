#include "grid2D.h"
#include "flatland_map.h"
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
		("makespan", po::value<int>()->default_value(INT_MAX), "Maximum makespan")
		("disjoint,d", po::value<bool>()->default_value(false), "disjoint splitting")
		("cutoffTime,t", po::value<int>()->default_value(300), "cutoff time (seconds)")
		("seed", po::value<int>()->default_value(0), "random seed")
		("debug", po::value<bool>()->default_value(false), "debug")
	;

	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);

	if (vm.count("help")) {
		std::cout << desc << std::endl;
		return 1;
	}

	po::notify(vm);

	// Grid2D G;
    FlatlandMap G;
	if (!G.load_map(vm["map"].as<string>()))
	{
	    cerr << "Fail to load the map" << endl;
	    return -1;
	}
	if (!G.load_agents(vm["agents"].as<string>()))
    {
        cerr << "Fail to load the agents" << endl;
        return -1;
    }
	G.preprocessing_heuristics();

	srand(vm["seed"].as<int>());

	ECBSSearch<FlatlandMap> ecbs(G, vm["weight"].as<double>(),
        vm["makespan"].as<int>(), vm["disjoint"].as<bool>(), vm["cutoffTime"].as<int>());
	// ECBSSearch<Grid2D> ecbs(G, vm["weight"].as<double>(), vm["disjoint"].as<bool>(), vm["cutoffTime"].as<int>());
	ecbs.screen = 0;
	if (vm["debug"].as<bool>())
        ecbs.screen = 1;
	ecbs.runECBSSearch();

	// test the solution, only needed for debug
	if (vm["debug"].as<bool>() && !ecbs.evaluateSolution())
    {
	    cout << "The solution is not feasible!" << endl;
	    return -1;
    }

	// ecbs.saveResults(vm["output"].as<string>(), vm["agents"].as<string>());
    ofstream stats;
    stats.open(vm["output"].as<string>(), std::ios::app);
    for (const auto& path: ecbs.get_solution())
    {
        stats << *path << endl;
    }

    stats.close();
	return 0;

}
