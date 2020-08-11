#pragma once
#include <boost/heap/fibonacci_heap.hpp>
#include <list>
#include <functional>  // for std::hash (c++11 and above)
#include <memory>
#include "common.h"
#include <cmath>
using namespace std;
using boost::heap::fibonacci_heap;
using boost::heap::compare;





class LLNode
{
public:

	int loc;
	int g_val;
	float h_val = 0;
	int heading;
	int actionToHere = 4;
	std::vector<int> possible_next_heading;
	LLNode* parent=nullptr;
	int timestep = 0;
	int time_generated=0;
	int show_time = 0;
	int num_internal_conf = 0; 
	bool in_openlist = false;
	// bool in_focallist = false;
	int next_malfunction = -1;
	int malfunction_left = 0;

	float speed = 1.0;
	float position_fraction = 0.0;
	int exit_loc=-1;
	int exit_heading=-1;
    bool active = 0;


	// the following is used to comapre nodes in the OPEN list
	struct compare_node 
	{
		// returns true if n1 > n2 (note -- this gives us *min*-heap).
		bool operator()(const LLNode* n1, const LLNode* n2) const 
		{
			if (n1->g_val + n1->h_val == n2->g_val + n2->h_val)
				return n1->g_val <= n2->g_val;  // break ties towards larger g_vals
			return n1->g_val + n1->h_val >= n2->g_val + n2->h_val;
		}
	};  // used by OPEN (heap) to compare nodes (top of the heap has min f-val, and then highest g-val)

	// the following is used to comapre nodes in the FOCAL list
	struct secondary_compare_node
	{
		bool operator()(const LLNode* n1, const LLNode* n2) const // returns true if n1 > n2
		{
			if (n1->num_internal_conf == n2->num_internal_conf)
			{
			    if (n1->show_time == n2->show_time) {
                    if (n1->h_val == n2->h_val) {
                        return (n1->g_val <= n2->g_val);
//                    if (rand() % 2 == 0)
//                        return true;
//                    else
//                        return false;

                    }
                    return n1->h_val >= n2->h_val;
                }
			    return n1->show_time <= n2->show_time;
			}
			return n1->num_internal_conf >= n2->num_internal_conf;  // n1 > n2 if it has more conflicts
		}
	};  // used by FOCAL (heap) to compare nodes (top of the heap has min number-of-conflicts)


	// define a typedefs for handles to the heaps (allow up to quickly update a node in the heap)
	typedef boost::heap::fibonacci_heap< LLNode*, compare<LLNode::compare_node> >::handle_type open_handle_t;
	typedef boost::heap::fibonacci_heap< LLNode*, compare<LLNode::secondary_compare_node> >::handle_type focal_handle_t;
	open_handle_t open_handle;
	focal_handle_t focal_handle;


	LLNode();
	LLNode(const LLNode& other);
	LLNode(int loc, int g_val, int h_val, LLNode* parent, int timestep,
		int num_internal_conf = 0, bool in_openlist = false);
	inline double getFVal() const { return g_val + h_val; }

	// The following is used by googledensehash for checking whether two nodes are equal
	// we say that two nodes, s1 and s2, are equal if
	// both are non-nullptr and agree on the id and timestep and same heading
	struct eqnode 
	{
		bool operator()(const LLNode* s1, const LLNode* s2) const 
		{

			return (s1 == s2) || (s1 && s2 &&
				s1->loc == s2->loc &&
				s1->timestep == s2->timestep &&
				s1->heading == s2->heading &&
				s1->malfunction_left == s2->malfunction_left &&
				round(s1->position_fraction*1000) == round(s2->position_fraction*1000) &&
                    s1->exit_loc == s2->exit_loc)
                ;
		}
	};

	// The following is used by googledensehash for generating the hash value of a nodes
	struct NodeHasher 
	{
		std::size_t operator()(const LLNode* n) const 
		{
			size_t loc_hash = std::hash<int>()(n->loc);
			size_t timestep_hash = std::hash<int>()(n->timestep);
			size_t heading = std::hash<int>()(n->heading);
			

			return (loc_hash ^ (timestep_hash << 1)*(heading << 1));
		}
	};

};

