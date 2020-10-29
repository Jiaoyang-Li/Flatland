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
	// define typedefs and handles for heap and hash_map
	typedef boost::heap::pairing_heap< LLNode*, boost::heap::compare<LLNode::compare_node> > heap_open_t;
    typedef boost::unordered_set<LLNode*, LLNode::NodeHasher, LLNode::eqnode> hashtable_t;
	heap_open_t open_list;
	hashtable_t allNodes_table;

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


	// Updates the path datamember
	void updatePath(LLNode* goal);

	// find path by time-space A* search
	// Returns true if a collision free path found  while
	// minimizing the number of internal conflicts (that is conflicts with known_paths for other agents found so far).
	// lowerbound is the lowerbound of the length of the path
	// max_plan_len used to compute the size of res_table
	bool search(bool flat = false);

	inline void releaseClosedListNodes(hashtable_t* allNodes_table);

	int getHeuristicAtStart() const {return (int)(my_heuristic[start_location].get_hval(agent.heading)); }

    SinglePlanning(const FlatlandLoader& ml, AgentsLoader& al, double f_w, float time_limit, options option);


};


