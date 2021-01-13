/*
 * @author: Team An_old_driver
 * @created: 09-2020
 * Copyright (c) 2020 The University of Southern California. All Rights Reserved.
 * Copyrights licensed under an Academic/non-profit use license.
 * See the accompanying LICENSE file for terms.
*/


#include <fstream>
#include <string>
#include <vector>
#include <boost/python.hpp>
#include <pthread.h>
#include "flat_map_loader.h"
#include "LNS.h"
#include "MCP.h"
#include "CPR.h"
#include "action_converter.h"

using namespace std::chrono;
typedef std::chrono::high_resolution_clock Time;
typedef std::chrono::duration<float> fsec;


namespace p = boost::python;

struct statistics {
    double runtime;
    int iterations;
    int sum_of_costs;
    int makespan;
    double initial_runtime;
    int initial_sum_of_costs;
    int initial_makespan;
    int unfinished_agents;

    int HL_num_expanded = 0;
    int HL_num_generated = 0;
    int LL_num_expanded = 0;
    int LL_num_generated = 0;
};

// for p thread call non-static function in class
struct wrap {
    float hard_time_limit;
    float soft_time_limit;
    float success_rate;
    int max_iterations;
    LNS& ins;

    wrap(float hard_time_limit, float soft_time_limit, float success_rate, int max_iterations, LNS& f ) :
        hard_time_limit(hard_time_limit), soft_time_limit(soft_time_limit), success_rate(success_rate),
        max_iterations(max_iterations), ins(f) {}
};

extern "C" void* call_func( void *f )
{
    std::unique_ptr< wrap > w( static_cast< wrap* >( f ) );
    w->ins.run(w->hard_time_limit, w->soft_time_limit, w->success_rate, w->max_iterations);

    return 0;
}

template <class Map>
class PythonCBS {
public:
	PythonCBS(p::object railEnv1, string framework, float soft_time_limit,
              int default_group_size, bool debug, bool replan,int stop_threshold,int agent_priority_strategy, int neighbor_generation_strategy);
	~PythonCBS(){
	    delete this->al;

        for (int i =0;  i< al_pool.size();i++){
            delete lns_pool[i];
	        if (i == this->best_thread_id)
	            continue;
	        delete al_pool[i];
	    }

        delete ml;
        if (cpr != nullptr)
        {
            delete cpr;
            cpr = nullptr;
        }

    }

	p::list getResult();

	bool search(float success_rate = 1.1, int max_iterations = 5000);
	p::dict getResultDetail();
	void updateAgents(p::object railEnv1);
    bool findConflicts() const;
    void buildMCP(void)
    {
        if (framework != "CPR")
            mcp.build(al, ml, options1);
    }
    void clearMCP(void) { mcp.clear(); }
    void printAllMCP(void) { mcp.printAll(); }
    void printMCP(int loc) { mcp.print(loc); }
    void printAgentTime(void) { mcp.printAgentTime(); }
    void printAgentNoWaitTime(void) { mcp.printAgentNoWaitTime(); }
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
               "destroy heuristic," <<
               "normalized cost," <<
               "dead agents," <<
               "HL nodes," <<
               "LL nodes," << endl;

        for (const auto & iteration_stat : iteration_stats) {
            for (const auto &data : iteration_stat) {
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
                       get<10>(data) <<"," << endl;
            }
        }
        output.close();
    }
    p::dict getActions(p::object railEnv1, int timestep, float time_limit);

    std::unordered_map<int,vector<hvals>> existing_heuristics; //goal heuristic

private:
	string framework;
	p::object railEnv;
	FlatlandLoader* ml;  // TODO:: Shouldn't it be Map* ml?
	AgentsLoader* al;
	float malfunction_rate;
	int max_timestep;
	MCP mcp;
    ActionConverter action_converter;
	vector<AgentsLoader*> al_pool;
	vector<LNS*> lns_pool;
	options options1;
    float hard_time_limit = 590;
    float soft_time_limit;
	int best_thread_id = 0;
	int best_initisl_priority_strategy = -1;
    int best_neighbour_strategy = -1;
    int default_group_size; // max number of agents in a group

    //default params
    int neighbor_generation_strategy = 3;  // 0: random walk; 1: start; 2: intersection; 3: adaptive; 4: iterative
    int agent_priority_strategy = 3;  // choose a number between 0 and 5. Suggest 1
                                        // 0: keep the original ordering
                                        // 1: prefer max speed then max distance
                                        // 2: prefer min speed then max distance
                                        // 3: prefer max speed then min distance
                                        // 4: prefer min speed then min distance
    int prirority_ordering_strategy = 0;  //0: random; 1: max regret; for replanning
    int replan_strategy = 1;
    bool replan_on = false;
    int max_replan_times = 50000;
    float max_replan_runtime = 100; // seconds

    //parallel parameters
    int strategies[4] = {1,3,5,6};
    int neighbours[4] = {0,2,3,4};
    int stop_threshold;

    int replan_times = 0;
    float replan_runtime = 0;
    vector<int> curr_locations;
    vector<int> prev_locations;

	//stats about CBS
    Time::time_point start_time;
    float runtime;
    vector<statistics> statistic_list;



    //replan
    list<int> to_be_replanned;
    // CPR
    CPR* cpr = nullptr;


    //stats about each iteration
    typedef tuple<int, double, double, double, int,
                    int, int, double, int, int, int> IterationStats;
    vector<list<IterationStats>> iteration_stats;

    bool parallel_LNS(int no_threads = 4, float success_rate = 1.1, int max_iterations = 5000);

    void generateNeighbor(int agent_id, const PathEntry& start, int start_time,
            set<int>& neighbor, int neighbor_size, int upperbound);

    void replan(const p::object& railEnv1, int timestep, float time_limit);
};


