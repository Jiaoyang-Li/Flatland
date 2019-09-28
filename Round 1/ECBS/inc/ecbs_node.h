#pragma once
#include "node.h"

class ECBSNode 
{
public:
	int agent_id;
	Constraint constraint; 
	list<std::shared_ptr<Conflict>> conflicts;

	ECBSNode* parent = NULL;
	list<tuple<int, vector<pathEntry>, int, int>> paths_updated; // agent id + new path + cost + lower bound
	int g_val;  // (total cost)
	int num_of_collisions;
	int sum_min_f_vals;  // saves the overall sum of min f-vals.

	uint64_t time_expanded;
	uint64_t time_generated;


	// the following is used to comapre nodes in the OPEN list
	struct compare_node 
	{
		bool operator()(const ECBSNode* n1, const ECBSNode* n2) const
		{
			return n1->sum_min_f_vals >= n2->sum_min_f_vals;
		}
	};  // used by OPEN to compare nodes by sum_min_f_vals (top of the heap has min sum_min_f_vals)

	// the following is used to comapre nodes in the FOCAL list
	struct secondary_compare_node 
	{
		bool operator()(const ECBSNode* n1, const ECBSNode* n2) const 
		{
			if (n1->num_of_collisions == n2->num_of_collisions)
				return n1->g_val >= n2->g_val;  // break ties towards shorter (overall) solutions
			return n1->num_of_collisions >= n2->num_of_collisions;
		}
	};  // used by FOCAL to compare nodes by num_of_collisions (top of the heap has min h-val)

	typedef boost::heap::fibonacci_heap< ECBSNode* , boost::heap::compare<compare_node> >
		::handle_type open_handle_t;
	typedef boost::heap::fibonacci_heap< ECBSNode* , boost::heap::compare<secondary_compare_node> >
		::handle_type focal_handle_t;

	open_handle_t open_handle;
	focal_handle_t focal_handle;

	// The following is used by googledensehash for generating the hash value of a nodes
	// this is needed because otherwise we'll have to define the specilized template inside std namespace
	struct ECBSNodeHasher
	{
		std::size_t operator()(const ECBSNode* n) const
		{
			size_t agent_id_hash = std::hash<size_t>()(n->agent_id);
			size_t time_generated_hash = std::hash<uint64_t>()(n->time_generated);
			return ( agent_id_hash ^ (time_generated_hash << 1) );
		}
	};

	ECBSNode() {}
	ECBSNode(int agent_id, ECBSNode* parent, int g_val, int num_of_collisions, int time_expanded, int sum_min_f_vals):
		agent_id(agent_id), parent(parent), g_val(g_val), num_of_collisions(num_of_collisions), 
		sum_min_f_vals(sum_min_f_vals), time_expanded(time_expanded){}
};


