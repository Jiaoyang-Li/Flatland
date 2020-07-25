#include <fstream>
#include <string>
#include <vector>
#include <boost/python.hpp>
#include <boost/thread.hpp>
#include "flat_map_loader.h"
#include "LNS.h"


namespace p = boost::python;

struct statistics {
    double runtime;
    double runtime_corridor=0;
    int HL_num_expanded = 0;
    int HL_num_generated = 0;
    int LL_num_expanded = 0;
    int LL_num_generated = 0;
    int num_standard = 0;
    int num_corridor2 = 0;
    int num_corridor = 0;
    int num_start = 0;
    int num_chasing = 0;
};

template <class Map>
class PythonCBS {
public:
	PythonCBS(p::object railEnv1, string framework, std::string algo, int t,
              int default_group_size, int debug, float f_w, int corridor,bool chasing, bool accept_partial_solution,
              int agent_priority_strategy, int neighbor_generation_strategy,
              int prirority_ordering_strategy, int replan_strategy);

	p::list getResult();

	int defaultGroupSize; // max number of agents in a group
    bool accept_partial_solution;
    int agent_priority_strategy;
	bool search();
	p::dict getResultDetail(int thread_id = 0);
	void updateAgents(p::object railEnv1);
	void updateFw(float fw);
    p::list benchmarkSingleGroup(int group_size,int iterations, int time_limit);
    p::list benchmarkSingleGroupLNS(int group_size,int iterations, int time_limit);
    bool findConflicts() const;

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
        for (const auto& data : iteration_stats[best_thread_id])
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
	string framework;
	p::object railEnv;
	FlatlandLoader* ml;  // TODO:: Shouldn't it be Map* ml?
	AgentsLoader* al;
	vector<AgentsLoader*> al_pool;
	constraint_strategy s;
	options options1;
	int timeLimit;
	int kRobust;
	int max_malfunction;
	float f_w;
	// MultiMapICBSSearch<Map>* icbs = NULL;
	int corridor_option = 0;
	bool corridor2=false;
	bool corridor4=false;
	bool trainCorridor1 = false;
	bool trainCorridor2 = false;
	bool chasing = false;
	int best_thread_id = 0;
    int neighbor_generation_strategy;
    int prirority_ordering_strategy;
    int replan_strategy;

	//stats about CBS
    std::clock_t start_time;
    double runtime;
    vector<statistics> statistic_list;


    //stats about each iteration
    typedef tuple<int, double, double, double, int,
                    int, int, int, int, int, int> IterationStats;
    vector<list<IterationStats>> iteration_stats;

    bool PrioritizedPlaning(AgentsLoader* al = NULL, int thread_id = 0, int priority_strategy = -1);
    bool GroupPrioritizedPlaning();
    bool parallel_LNS(int no_threads = 4);

    void generateNeighbor(int agent_id, const PathEntry& start, int start_time,
            set<int>& neighbor, int neighbor_size, int upperbound, AgentsLoader * al = NULL);
    void updateCBSResults(const MultiMapICBSSearch<Map>& cbs, int thread_id = 0)
    {
        statistic_list[thread_id].runtime = (double)(std::clock() - start_time) / CLOCKS_PER_SEC;
        statistic_list[thread_id].runtime_corridor += cbs.runtime_corridor/CLOCKS_PER_SEC;
        statistic_list[thread_id].HL_num_expanded += cbs.HL_num_expanded;
        statistic_list[thread_id].HL_num_generated += cbs.HL_num_generated;
        statistic_list[thread_id].LL_num_expanded += cbs.LL_num_expanded;
        statistic_list[thread_id].LL_num_generated += cbs.LL_num_generated;
        statistic_list[thread_id].num_standard += cbs.num_standard;
        statistic_list[thread_id].num_corridor2 += cbs.num_corridor2;
        statistic_list[thread_id].num_corridor += cbs.num_corridor;
        statistic_list[thread_id].num_start+=cbs.num_start;
        statistic_list[thread_id].num_chasing += cbs.num_chasing;
    }

    bool hasConflicts(const vector<Path>& paths) const;
};


