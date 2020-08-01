#include <fstream>
#include <string>
#include <vector>
#include <boost/python.hpp>
#include <pthread.h>
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

// for p thread call non-static function in class
struct wrap {
    float hard_time_limit;
    float soft_time_limit;
    LNS& ins;

    wrap(float hard_time_limit, float soft_time_limit, LNS& f ) :
        hard_time_limit(hard_time_limit), soft_time_limit(soft_time_limit), ins(f) {}
};

extern "C" void* call_func( void *f )
{
    std::unique_ptr< wrap > w( static_cast< wrap* >( f ) );
    w->ins.run(w->hard_time_limit, w->soft_time_limit);

    return 0;
}

template <class Map>
class PythonCBS {
public:
	PythonCBS(p::object railEnv1, string framework, std::string algo, float soft_time_limit,
              int default_group_size, int debug, float f_w, int corridor,bool chasing, bool accept_partial_solution,
              int agent_priority_strategy, int neighbor_generation_strategy,
              int prirority_ordering_strategy, int replan_strategy);
	~PythonCBS(){
	    delete this->al;

        for (int i =0;  i< al_pool.size();i++){
            delete lns_pool[i];
	        if (i == this->best_thread_id)
	            continue;
	        delete al_pool[i];
	    }

        delete ml;

    }

	p::list getResult();

	int defaultGroupSize; // max number of agents in a group
    bool accept_partial_solution;
    int agent_priority_strategy;
	bool search();
	p::dict getResultDetail();
	void updateAgents(p::object railEnv1);
	void updateFw(float fw);
    p::list benchmarkSingleGroup(int group_size,int iterations, int time_limit);
    p::list benchmarkSingleGroupLNS(int group_size,int iterations, int time_limit);
    bool findConflicts() const;
    p::list getNextLoc(p::list agent_location, int timestep);
    void updateMCP(p::list agent_location, p::dict agent_action);
    void buildMCP(void);
    void clearMCP(void) { mcp.clear(); };
    void printAllMCP(void);
    void printMCP(int loc);
    void printAgentTime(void);

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
               "LL nodes," <<
               "Initial agent priority"<<
               endl;
        for (int i = 0 ; i<iteration_stats.size(); i++) {
            for (const auto &data : iteration_stats[i]) {
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
                       get<10>(data) <<"," <<
                       strategies[i] << endl;
            }
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
	vector<LNS*> lns_pool;
	constraint_strategy s;
	options options1;
    float hard_time_limit = 240;
    float soft_time_limit;
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
	int bset_initisl_priority_strategy = -1;
    int neighbor_generation_strategy;
    int prirority_ordering_strategy;
    int replan_strategy;

	//stats about CBS
    std::clock_t start_time;
    double runtime;
    vector<statistics> statistic_list;
    int strategies[4] = {0,1,3,5};



    //stats about each iteration
    typedef tuple<int, double, double, double, int,
                    int, int, double, int, int, int> IterationStats;
    vector<list<IterationStats>> iteration_stats;

    bool PrioritizedPlaning(AgentsLoader* al = NULL, int thread_id = 0, int priority_strategy = -1);
    bool GroupPrioritizedPlaning();
    bool parallel_LNS(int no_threads = 4);

    void generateNeighbor(int agent_id, const PathEntry& start, int start_time,
            set<int>& neighbor, int neighbor_size, int upperbound);

    // MCP
    typedef list<tuple<int, int>> Occupy;
    vector<Occupy> mcp;
    vector<int> agent_time;
    vector<int> to_go;
    vector<int> appear_time;

    void updateCBSResults(const MultiMapICBSSearch<Map>& cbs, int thread_id = 0)
    {
        statistic_list[thread_id].runtime_corridor += cbs.runtime_corridor.count();
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

    bool hasConflicts() const;
};


