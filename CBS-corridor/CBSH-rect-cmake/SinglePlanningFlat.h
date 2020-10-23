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
#include <boost/heap/fibonacci_heap.hpp>
#include <boost/unordered_set.hpp>
using namespace std::chrono;
typedef std::chrono::high_resolution_clock Time;
typedef std::chrono::duration<float> fsec;


class SinglePlanningFlat
{
public:
	// define typedefs and handles for heap and hash_map
	typedef boost::heap::fibonacci_heap< LLNode*, boost::heap::compare<LLNode::compare_node> > heap_open_t;
	typedef boost::heap::fibonacci_heap< LLNode*, boost::heap::compare<LLNode::secondary_compare_node> > heap_focal_t;
    typedef boost::unordered_set<LLNode*, LLNode::NodeHasher, LLNode::eqnode> hashtable_t;
	heap_open_t open_list;
	heap_focal_t focal_list;
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

	double focal_threshold;  // FOCAL's lower bound ( = e_weight * min_f_val)
	double min_f_val;  // min f-val seen so far
	int num_of_conf; // number of conflicts between this agent to all the other agents


	//Checks if a vaild path found (wrt my_map and constraints)
	//Note -- constraint[timestep] is a list of pairs. Each pair is a disallowed <loc1,loc2> (loc2=-1 for vertex constraint).
	//bool isConstrained(int curr_id, int next_id, int next_timestep, const std::vector< std::list< std::pair<int, int> > >* cons) const;
	inline bool isConstrained(int curr_id, int next_id, int next_timestep, const std::vector< std::list< std::pair<int, int> > >* cons)  const
	{
		if (cons == nullptr)
			return false;
		// check vertex constraints (being in next_id at next_timestep is disallowed)
		if (next_timestep < static_cast<int>(cons->size()))
		{
			for (const auto & it : cons->at(next_timestep))
			{
				if ((std::get<0>(it) == next_id && std::get<1>(it) < 0)//vertex constraint
					|| (std::get<0>(it) == curr_id && std::get<1>(it) == next_id)) // edge constraint
					return true;
			}
		}
		return false;
	}

	// Updates the path datamember
	void updatePath(LLNode* goal);

	// find path by time-space A* search
	// Returns true if a collision free path found  while
	// minimizing the number of internal conflicts (that is conflicts with known_paths for other agents found so far).
	// lowerbound is the lowerbound of the length of the path
	// max_plan_len used to compute the size of res_table
	bool search();

	inline void releaseClosedListNodes(hashtable_t* allNodes_table);

	int getHeuristicAtStart() const {return (int)(my_heuristic[start_location].get_hval(agent.heading) / agent.speed); }

    SinglePlanningFlat(const FlatlandLoader& ml, AgentsLoader& al, double f_w, float time_limit, options option);


};


