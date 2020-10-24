#include <fstream>
#include <string>
#include <vector>
#include <boost/python.hpp>
#include <pthread.h>
#include "flat_map_loader.h"
#include "LNS.h"
#include "MCP.h"
#include "CPR.h"

using namespace std::chrono;
typedef std::chrono::high_resolution_clock Time;
typedef std::chrono::duration<float> fsec;


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
        if (cpr != nullptr)
        {
            delete cpr;
            cpr = nullptr;
        }
    }

	p::list getResult();

	int defaultGroupSize; // max number of agents in a group
    bool accept_partial_solution;
    int agent_priority_strategy;
	bool search();
	p::dict getResultDetail();
	void replan(p::object railEnv1, int timestep, float time_limit);
	void updateFw(float fw);
	void updateAgents(p::object railEnv1);
    p::list benchmarkSingleGroup(int group_size,int iterations, int time_limit);
    p::list benchmarkSingleGroupLNS(int group_size,int iterations, int time_limit);
    bool findConflicts() const;
    p::list getNextLoc(p::list agent_location, int timestep)
    {
        boost::python::list next_loc;
        if (framework == "CPR")
        {
            vector<int> to_go(al->getNumOfAllAgents(), -1);
            cpr->getNextLoc(to_go);
            for (int i = 0; i < al->getNumOfAllAgents(); i++)
                next_loc.append(to_go[i]);
            return next_loc;
        }
        else
        {
            mcp.getNextLoc(agent_location,timestep);
            for (int i = 0; i < al->getNumOfAllAgents(); i++)
                next_loc.append(mcp.to_go[i]);
            al->updateAgents(railEnv.attr("agents"));
            return next_loc;
        }
    }
    void updateMCP(p::list agent_location, p::dict agent_action)
    {
        if (framework == "CPR")
        {
            cpr->update(agent_location);
        }
        else
        {

            mcp.update(agent_location, agent_action);
        }

    }
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
               "LL nodes,";
        if (framework == "Parallel-LNS")
            output << "Initial agent priority" << endl;
        else if(framework == "Parallel-Neighbour-LNS")
            output <<"Neighbour strategy"<< endl;
        else
            output<<endl;

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
                       get<10>(data) <<"," ;
                if (framework == "Parallel-LNS")
                    output << strategies[i] << ","<< endl;
                else if(framework == "Parallel-Neighbour-LNS")
                    output << neighbours[i] <<","<<endl;
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
	MCP mcp;
	vector<AgentsLoader*> al_pool;
	vector<LNS*> lns_pool;
	constraint_strategy s;
	options options1;
    float hard_time_limit = 280;
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
	int best_initisl_priority_strategy = -1;
    int best_neighbour_strategy = -1;
    int neighbor_generation_strategy;
    int prirority_ordering_strategy;
    int replan_strategy;

	//stats about CBS
    Time::time_point start_time;
    float runtime;
    vector<statistics> statistic_list;
    int strategies[4] = {0,1,3,5};
    int neighbours[4] = {0,2,3,4};

    //replan
    list<int> to_be_replanned;
    // CPR
    CPR* cpr = nullptr;

    //stats about each iteration
    typedef tuple<int, double, double, double, int,
                    int, int, double, int, int, int> IterationStats;
    vector<list<IterationStats>> iteration_stats;

    bool PrioritizedPlaning(AgentsLoader* al = NULL, int thread_id = 0, int priority_strategy = -1);
    bool GroupPrioritizedPlaning();
    bool parallel_LNS(int no_threads = 4);
    bool parallel_neighbour_LNS(int no_threads = 4);

    void generateNeighbor(int agent_id, const PathEntry& start, int start_time,
            set<int>& neighbor, int neighbor_size, int upperbound);

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


