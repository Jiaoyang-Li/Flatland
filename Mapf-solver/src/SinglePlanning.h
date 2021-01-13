/*
 * @author: Team An_old_driver
 * @created: 09-2020
 * Copyright (c) 2020 The University of Southern California. All Rights Reserved.
 * Copyrights licensed under an Academic/non-profit use license.
 * See the accompanying LICENSE file for terms.
*/

#pragma once

#include <stdlib.h>

#include <vector>
#include <list>
#include <utility>
#include <ctime>
#include "common.h"
#include "agents_loader.h"
#include "LLNode.h"
#include "flat_map_loader.h"
#include "ConstraintTable.h"
#include "compute_heuristic.h"
#include <boost/heap/pairing_heap.hpp>
#include <boost/unordered_set.hpp>
using namespace std::chrono;
typedef std::chrono::high_resolution_clock Time;
typedef std::chrono::duration<float> fsec;


class SinglePlanning
{
public:
    Agent& agent;
	int start_location;
	int goal_location;

    vector<PathEntry> path;

    AgentsLoader& al;
	const FlatlandLoader& ml;
	int map_size;
	int num_col;
	const std::vector<hvals>& my_heuristic;  // this is the precomputed heuristic for this agent
    ConstraintTable& constraintTable;

    int screen;
    float time_limit;
	float f_w;

	uint64_t LL_num_expanded;
	uint64_t LL_num_generated;


	bool search(bool flat = false);

	int getHeuristicAtStart() const {return (int)(my_heuristic[start_location].get_hval(agent.heading)); }

    SinglePlanning(const FlatlandLoader& ml, AgentsLoader& al, double f_w, float time_limit, options option);

private:
    // define typedefs and handles for heap and hash_map
    typedef boost::heap::pairing_heap< LLNode*, boost::heap::compare<LLNode::compare_node> > heap_open_t;
    typedef boost::unordered_set<LLNode*, LLNode::NodeHasher, LLNode::eqnode> hashtable_t;
    heap_open_t open_list;
    hashtable_t allNodes_table;
    // Updates the path datamember
    void updatePath(LLNode* goal);
    inline void releaseClosedListNodes()
    {
        for (auto it : allNodes_table)
            delete it;
        open_list.clear();
        allNodes_table.clear();
    }
};


class SIPP: public SinglePlanning
{
public:
    bool search();
    SIPP(const FlatlandLoader& ml, AgentsLoader& al, double f_w, float time_limit, options option):
            SinglePlanning(ml, al, f_w, time_limit, option) {}

private:

    void getSafeIntervals(int prev_loc, int prev_timestep,
                          const Interval& prev_interval, int next_loc, int next_h, list<Interval>& intervals);
    // define typedefs and handles for heap and hash_map
    typedef boost::heap::pairing_heap< SIPPNode*, boost::heap::compare<LLNode::compare_node> > heap_open_t;
    typedef boost::unordered_set<SIPPNode*, SIPPNode::NodeHasher, SIPPNode::eqnode> hashtable_t;
    heap_open_t open_list;
    hashtable_t allNodes_table;
    // Updates the path datamember
    void updatePath(SIPPNode* goal);
    inline void releaseClosedListNodes()
    {
        for (auto it : allNodes_table)
            delete it;
        open_list.clear();
        allNodes_table.clear();
    }
};

