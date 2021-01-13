/*
 * @author: Team An_old_driver
 * @created: 09-2020
 * Copyright (c) 2020 The University of Southern California. All Rights Reserved.
 * Copyrights licensed under an Academic/non-profit use license.
 * See the accompanying LICENSE file for terms.
*/
#pragma once
#include "SinglePlanning.h"
#include <chrono>
#include <atomic>
using namespace std::chrono;
typedef std::chrono::high_resolution_clock Time;
typedef std::chrono::duration<float> fsec;


#define DEFAULT_GROUP_SIZE 5
class LNS
{
public:
    std::atomic<int>* complete= nullptr ;

    bool pp_only = false;
    bool skip_pp = false;
    bool dead_agent = false; // dead agent is detected
    float runtime = 0;
    float initial_runtime = 0;
    int sum_of_costs = 0;
    int initial_sum_of_costs = 0;
    int makespan = 0;
    int initial_makespan = 0;
    int iterations = 0;
    int replan_times = 0;
    vector<int> neighbors;
    //stats about each iteration
    typedef tuple<int, double, double, double, int,
            int, int, double, int, int, int> IterationStats;
    list<IterationStats> iteration_stats;

    LNS(AgentsLoader& al, FlatlandLoader& ml, double f_w,  int agent_priority_strategy,
        const options& options1,int max_group_size,
        int neighbor_generation_strategy,int prirority_ordering_strategy, int replan_strategy, int stop_threshold):
            al(al), ml(ml), f_w(f_w), agent_priority_strategy(agent_priority_strategy), options1(options1),
            max_group_size(max_group_size),
            destroy_strategy(neighbor_generation_strategy),
            prirority_ordering_strategy(prirority_ordering_strategy),
            replan_strategy(replan_strategy),stop_threshold(stop_threshold) {
        max_timestep = al.constraintTable.length_max;
    }
    bool run(float hard_time_limit, float soft_time_limit, float success_rate = 1.1, int max_iterations = 5000);
    bool replan(float time_limit);
    bool replan(list<int>& to_be_replanned, float time_limit);
    bool getInitialSolution(float success_rate = 1.1,int max_iterations =5000);
    void set_complete(std::atomic<int>* complete){this->complete = complete;}
private:
    high_resolution_clock::time_point start_time;
    AgentsLoader& al;
    FlatlandLoader& ml;
    double f_w;
    int agent_priority_strategy;
    options options1;
    int max_timestep;
    //data for neighbors
    list<Path> neighbor_paths;
    int neighbor_sum_of_costs = 0;
    int neighbor_sum_of_showup_time = 0;
    int neighbor_makespan = 0;
    int delta_costs = 0;
    int max_group_size = DEFAULT_GROUP_SIZE;

    int stop_threshold = 0;

    vector<int> intersections;
    map<int, list<int>> start_locations;  // <start location, corresponding agents>

    // intput params
    float hard_time_limit = 0;
    float soft_time_limit = 0;
    int destroy_strategy = 0; // 0: random walk; 1: start; 2: intersection
    int prirority_ordering_strategy = 0; // 0: random; 1: max regret
    int replan_strategy = 0; // 0: CBS; 1: prioritized planning

    bool adaptive_destroy = false;
    bool iterative_destroy = false;
    double decay_factor = 0.01;
    double reaction_factor = 0.1;
    vector<double> destroy_heuristics;


    void replanByPP();

    void generateNeighborByRandomWalk(boost::unordered_set<int>& tabu_list);
    bool generateNeighborByStart();
    bool generateNeighborByIntersection();
    bool generateNeighborByTemporalIntersection();

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


    void updateCBSResults(const SinglePlanning& planner)
    {
        runtime = ((fsec)(Time::now() - start_time)).count();
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
