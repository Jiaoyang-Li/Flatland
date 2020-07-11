#include <fstream>
#include <string>
#include <vector>
#include <boost/python.hpp>
#include "flat_map_loader.h"


#include "ICBSSearch.h"


namespace p = boost::python;

template <class Map>
class PythonCBS {
public:
	PythonCBS(p::object railEnv1, std::string algo, int t,
              int default_group_size, int debug, float f_w, int corridor,bool chasing, bool accept_partial_solution,
              int agent_priority_strategy);

	p::list getResult();

	int defaultGroupSize; // max number of agents in a group
    bool accept_partial_solution;
    int agent_priority_strategy;
	bool search();
	p::dict getResultDetail();
	void updateAgents(p::object railEnv1);
	void updateFw(float fw);


	void writeResultsToFile(const string& fileName) const
    {
        std::ofstream output;
        output.open(fileName);
        // header
        output << "group size," <<
               "time limit," <<
               "total runtime," <<
               "runtime," <<
               "total makespan," <<
               "solution cost," <<
               "sum of minimal time," <<
               "planned agents," <<
               "dead agents," <<
               "HL nodes," <<
               "LL nodes" << endl;
        for (const auto& data : iteration_stats)
        {
            output << get<0>(data) << "," <<
                   get<1>(data) << "," <<
                   get<2>(data) << "," <<
                   get<3>(data) << "," <<
                   get<4>(data) << "," <<
                   get<5>(data) << "," <<
                   get<6>(data) << "," <<
                   get<7>(data) << "," <<
                   get<8>(data) << "," <<
                   get<9>(data) << "," <<
                   get<10>(data) << endl;
        }
        output.close();
    }

private:
	std::string algo;
	p::object railEnv;
	FlatlandLoader* ml;  // TODO:: Shouldn't it be Map* ml?
	AgentsLoader* al;
	constraint_strategy s;
	options options1;
	int timeLimit;
	int kRobust;
	int max_malfunction;
	float f_w;
	// MultiMapICBSSearch<Map>* icbs = NULL;
	bool corridor2=false;
	bool corridor4=false;
	bool trainCorridor1 = false;
	bool trainCorridor2 = false;
	bool chasing = false;

	//stats about CBS
    std::clock_t start_time;
    double runtime;
    double runtime_corridor;
    int HL_num_expanded = 0;
    int HL_num_generated = 0;
    int LL_num_expanded = 0;
    int LL_num_generated = 0;
    int num_rectangle = 0;
    int num_corridor2 = 0;
    int num_semi_corridor = 0;
    int num_start = 0;
    int num_chasing = 0;

    ConstraintTable constraintTable;

    //stats about each iteration
    typedef tuple<int, double, double, double, int,
                    int, int, int, int, int, int> IterationStats;
    list<IterationStats> iteration_stats;

    void updateCBSResults(const MultiMapICBSSearch<Map>& cbs)
    {
        runtime = (double)(std::clock() - start_time) / CLOCKS_PER_SEC;
        runtime_corridor += cbs.runtime_corridor/CLOCKS_PER_SEC;
        HL_num_expanded += cbs.HL_num_expanded;
        HL_num_generated += cbs.HL_num_generated;
        LL_num_expanded += cbs.LL_num_expanded;
        LL_num_generated += cbs.LL_num_generated;
        num_rectangle += cbs.num_rectangle;
        num_corridor2 += cbs.num_corridor2;
        num_semi_corridor += cbs.num_semi_corridor;
        num_start+=cbs.num_start;
        num_chasing += cbs.num_chasing;
    }

    bool hasConflicts(const vector<Path>& paths) const;
};


