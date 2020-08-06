#pragma once
#include "ICBSSearch.h"
#include <chrono>
using namespace std::chrono;
typedef std::chrono::high_resolution_clock Time;
typedef std::chrono::duration<float> fsec;

#define DEFAULT_GROUP_SIZE 5
class LNS
{
public:
    //stats about CBS
    double runtime_corridor = 0;
    int HL_num_expanded = 0;
    int HL_num_generated = 0;
    int LL_num_expanded = 0;
    int LL_num_generated = 0;
    int num_standard = 0;
    int num_corridor2 = 0;
    int num_corridor = 0;
    int num_start = 0;
    int num_chasing = 0;
    bool pp_only = false;
    bool skip_pp = false;
    

    //stats about each iteration
    typedef tuple<int, double, double, double, int,
            int, int, double, int, int, int> IterationStats;
    list<IterationStats> iteration_stats;

    LNS(AgentsLoader& al, FlatlandLoader& ml, double f_w, const constraint_strategy c,
        int agent_priority_strategy,
        const options& options1,
        bool corridor2,
        bool trainCorridor1,
        bool chasing, int neighbor_generation_strategy,
        int prirority_ordering_strategy, int replan_strategy):
            al(al), ml(ml), f_w(f_w), c(c), agent_priority_strategy(agent_priority_strategy), options1(options1),
            corridor2(corridor2), trainCorridor1(trainCorridor1), chasing(chasing),
            destroy_strategy(neighbor_generation_strategy),
            prirority_ordering_strategy(prirority_ordering_strategy),
            replan_strategy(replan_strategy) {
        max_timestep = al.constraintTable.length_max;
    }
    bool run(float hard_time_limit, float soft_time_limit);

private:
    high_resolution_clock::time_point start_time;
    float runtime = 0;
    AgentsLoader& al;
    FlatlandLoader& ml;
    double f_w;
    constraint_strategy c;
    int agent_priority_strategy;
    options options1;
    int max_timestep;

    //data for neighbors
    vector<int> neighbors;
    list<Path> neighbor_paths;
    int neighbor_sum_of_costs = 0;
    int neighbor_sum_of_showup_time = 0;
    int neighbor_makespan = 0;
    int delta_costs = 0;
    int group_size = DEFAULT_GROUP_SIZE; // this is useful only when we use CBS to replan
    int max_group_size = DEFAULT_GROUP_SIZE;

    vector<int> intersections;
    map<int, list<int>> start_locations;  // <start location, corresponding agents>

    // intput params
    float hard_time_limit = 0;
    float soft_time_limit = 0;
    const bool& corridor2;
    const bool& trainCorridor1;
    const bool& chasing;
    int destroy_strategy = 0; // 0: random walk; 1: start; 2: intersection
    int prirority_ordering_strategy = 0; // 0: random; 1: max regret
    int replan_strategy = 0; // 0: CBS; 1: prioritized planning

    bool adaptive_destroy = false;
    bool iterative_destroy = false;
    double decay_factor = 0.01;
    double reaction_factor = 0.1;
    vector<double> destroy_heuristics;

    bool getInitialSolution();

    void replanByPP();
    bool replanByCBS();

    void generateNeighborByRandomWalk(boost::unordered_set<int>& tabu_list);
    bool generateNeighborByStart();
    bool generateNeighborByIntersection();

    void sortNeighborsRandomly();
    void sortNeighborsByRegrets();
    void sortNeighborsByStrategy();

    //tools
    void updateNeighborPaths();
    void updateNeighborPathsCosts();
    void addAgentPath(int agent, const Path& path);
    void deleteNeighborPaths();
    void quickSort(vector<int>& agent_order, int low, int high, bool regret);
    void randomWalk(int agent_id, const PathEntry& start, int start_timestep,
                    set<int>& neighbor, int neighbor_size, int upperbound);

    void updateCBSResults(const MultiMapICBSSearch<FlatlandLoader>& cbs)
    {
        runtime = ((fsec)(Time::now() - start_time)).count();
        runtime_corridor += cbs.runtime_corridor.count();
        HL_num_expanded += cbs.HL_num_expanded;
        HL_num_generated += cbs.HL_num_generated;
        LL_num_expanded += cbs.LL_num_expanded;
        LL_num_generated += cbs.LL_num_generated;
        num_standard += cbs.num_standard;
        num_corridor2 += cbs.num_corridor2;
        num_corridor += cbs.num_corridor;
        num_start+=cbs.num_start;
        num_chasing += cbs.num_chasing;
    }

    // bool hasConflicts(const vector<Path>& paths) const;


    inline bool compareByRegrets(int a1, int a2)
    {
        int r1 = (int)(al.paths_all[a1].size() -
                       al.agents_all[a1].distance_to_goal / al.agents_all[a1].speed);
        int r2 = (int)(al.paths_all[a2].size() -
                       al.agents_all[a2].distance_to_goal / al.agents_all[a2].speed);
        if (r1 == r2)
            return rand() % 2;
        else
            return r1  > r2;
    }
};
