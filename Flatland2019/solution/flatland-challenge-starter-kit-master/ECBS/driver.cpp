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
		("groupSize,g", po::value<int>()->default_value(16), "number of agents in each sub group")
		("makespan", po::value<int>()->default_value(INT_MAX), "Maximum makespan")
		("disjoint,d", po::value<bool>()->default_value(false), "disjoint splitting")
		("cutoffTime,t", po::value<int>()->default_value(10), "cutoff time (seconds)")
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
	// G.preprocessing_heuristics();
	G.generate_agent_order();

	srand(vm["seed"].as<int>());
	int defaultGroupSize = vm["groupSize"].as<int>();
	int groupSize = defaultGroupSize;
	while (G.generate_instance(groupSize))
	{
		ECBSSearch<FlatlandMap> ecbs(G, vm["weight"].as<double>(),
			vm["makespan"].as<int>(), vm["disjoint"].as<bool>(), vm["cutoffTime"].as<int>());
		// ECBSSearch<Grid2D> ecbs(G, vm["weight"].as<double>(), vm["disjoint"].as<bool>(), vm["cutoffTime"].as<int>());
		ecbs.screen = 0;
		if (vm["debug"].as<bool>())
			ecbs.screen = 1;
		ecbs.runECBSSearch();
		if (vm["debug"].as<bool>())
			ecbs.printResults();
		if (!ecbs.solution_found)
		{
			groupSize = groupSize / 2;
			continue;
		}
		G.update_paths(ecbs.paths);
		groupSize = defaultGroupSize;
	}

	// test the solution, only needed for debug
	if (vm["debug"].as<bool>())
	{
		ECBSSearch<FlatlandMap> ecbs(G, vm["weight"].as<double>(),
			vm["makespan"].as<int>(), vm["disjoint"].as<bool>(), vm["cutoffTime"].as<int>());
		ecbs.paths.resize(G.paths.size());
		for (size_t i = 0; i < G.paths.size(); i++)
			ecbs.paths[i] = &G.paths[i];
		if (ecbs.evaluateSolution())
			cout << "The solution is conflict-free!" << endl;
		else
		{
			cout << "The solution is not feasible!" << endl;
			return -1;
		}
	}

	// ecbs.saveResults(vm["output"].as<string>(), vm["agents"].as<string>());
    ofstream stats;
    stats.open(vm["output"].as<string>(), std::ios::out);
	int makespan = 0;
    for (const auto& path: G.paths)
    {
        stats << path << endl;
		if (!path.empty())
			makespan = max(makespan, path.back().timestep);
    }
    stats.close();

	if (vm["debug"].as<bool>())
	{
		cout << "Total makepsan is " << makespan << endl;
	}
	return 0;

}
