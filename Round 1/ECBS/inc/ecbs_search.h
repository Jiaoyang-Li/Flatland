// ECBS Search (High-level)
#pragma once
#include "single_agent_ecbs.h"
#include "ecbs_node.h"

template<class MyGraph>
class ECBSSearch
{
public:
	//settings
	double focal_w;
	 bool disjointSplitting;
	 int time_limit;
	 int screen = 0;

	 // statistics of efficiency
	 double runtime = 0; // not include preprocessing
	 int prepTime = 0; // CPU time for preprocessing
	 uint64_t HL_num_expanded = 0;
	 uint64_t HL_num_generated = 0;
	 uint64_t LL_num_expanded = 0;
	 uint64_t LL_num_generated = 0;  
	  
	  // statistics of solution quality
	  int min_sum_f_vals;
	  bool solution_found = false;
	  double solution_cost = -1;

	  ECBSSearch(const MyGraph& G, double focal_w, bool disjointSplitting, int cutoffTime);
	~ECBSSearch();
	  bool runECBSSearch();

	  // print
	  void printPaths() const;
	  void printResults() const;
	  void saveResults(const string& outputFile, const string& agentFile) const;

private:

	const MyGraph& G;
	CAT cat; // conflict-avoidance table

	SingleAgentPlanner single_planner;
	ECBSNode* dummy_start;
	vector <int> paths_costs_found_initially;
	vector <int> ll_min_f_vals_found_initially;  // contains initial ll_min_f_vals found
	vector < vector<pathEntry>* > paths_found_initially;  // contain initial paths found
	vector < vector<pathEntry>* > paths;  // agents paths
	vector <int> ll_min_f_vals;  // each entry [i] represent the lower bound found for agent[i]
	vector <int> paths_costs;

	typedef boost::heap::fibonacci_heap< ECBSNode*, boost::heap::compare<ECBSNode::compare_node> > heap_open_t;
	typedef boost::heap::fibonacci_heap< ECBSNode*, boost::heap::compare<ECBSNode::secondary_compare_node> > heap_focal_t;
	heap_open_t open_list;
	heap_focal_t focal_list;
	list<ECBSNode*> allNodes_table;
	double focal_list_threshold;

	// input
	size_t map_size;
	int num_of_agents;

	inline bool switchedLocations(int agent1_id, int agent2_id, size_t timestep);
	inline int getAgentLocation(int agent_id, size_t timestep);
	bool finfConflicts(ECBSNode& curr);
	void findConflicts(list<std::shared_ptr<Conflict>>& set, int a1, int a2);
	std::shared_ptr<Conflict>  chooseConflict(const list<std::shared_ptr<Conflict>>& conflicts);
	int getPathsMaxLength();
	vector < list< tuple<int, int, bool> > >* collectConstraints(ECBSNode* curr, int agent_id);
	bool generateChild(ECBSNode* child);
	void updateReservationTable(size_t max_plan_len, int exclude_agentconst);
	void updateFocalList(double old_lower_bound, double new_lower_bound, double f_weight);
	inline void updatePaths(ECBSNode* curr);
	bool findPathForSingleAgent(ECBSNode*  node, int ag);


};


