#pragma once

#include "ICBSNode.h"
#include "SingleAgentICBS.h"
#include "compute_heuristic.h"
#include "agents_loader.h"
#include "RectangleReasoning.h"
#include "CorridorReasoning.h"
#include "ConstraintTable.h"
#include "common.h"
#include "MDD.h"
#include <unordered_map>
#include <boost/python.hpp>

struct options {
	bool asymmetry_constraint;
	bool debug;
	bool ignore_t0;
	bool shortBarrier;
	bool flippedRec;
};



class ICBSSearch
{
public:
	double runtime = 0;
	double runtime_lowlevel;
	double runtime_conflictdetection;
	double runtime_computeh;
	double runtime_listoperation;
	double runtime_updatepaths;
	double runtime_updatecons;
	double RMTime = 0;

	

	ICBSNode* dummy_start;

	uint64_t HL_num_expanded = 0;
	uint64_t HL_num_generated = 0;
	uint64_t LL_num_expanded = 0;
	uint64_t LL_num_generated = 0;

	uint64_t num_corridor2 = 0;
	uint64_t num_corridor4 = 0;
	uint64_t num_rectangle = 0;
	uint64_t num_0FlipRectangle = 0;
	uint64_t num_1FlipRectangle = 0;
	uint64_t num_2FlipRectangle = 0;
	uint64_t num_target = 0;
	uint64_t num_standard = 0;
    uint64_t num_chasing = 0;
    uint64_t num_activeConflict = 0;



    bool solution_found = false;
	int solution_cost;
	tuple<int, int, int> min_f_val;  // <#dead agents, makespan, sum of costs>
    tuple<int, int, int> focal_list_threshold;  // <#dead agents, w * makespan, w * sum of costs>
    ICBSNode* goal_node = nullptr;
	bool cardinalRect = false;
	bool rectangleMDD = false;
	bool corridor2 = false;
	bool corridor4 = false;
	bool cardinalCorridorReasoning = false;
	bool targetReasoning=false;
	int kDelay;
	bool asymmetry_constraint;
	int numOfRectangle = 0;
	bool debug_mode=false;
	bool ignore_t0=false;
	bool shortBarrier = false;
	std::clock_t start;
	bool ignoreFinishedAgent = false;
	int max_malfunction = 5;

    vector<vector<PathEntry>*> paths;

	void printBT(const std::string& prefix, const ICBSNode* node, bool isLeft);
	void printHLTree();




	ICBSSearch() {};

protected:
	
	typedef boost::heap::fibonacci_heap< ICBSNode*, boost::heap::compare<ICBSNode::compare_node> > heap_open_t;
	typedef boost::heap::fibonacci_heap< ICBSNode*, boost::heap::compare<ICBSNode::secondary_compare_node> > heap_focal_t;
	heap_open_t open_list;
	heap_focal_t focal_list;
	list<ICBSNode*> allNodes_table;

	CorridorTable corridorTable;

	ConstraintTable constraintTable;

	constraint_strategy cons_strategy;
	int time_limit;
	double focal_w = 1.0;
	int screen;

	const bool* my_map;
	int map_size;
	int num_of_agents;
	const int* actions_offset;
	const int* moves_offset;
	int num_col;
	
	vector<vector<PathEntry>> paths_found_initially;  // contain initial paths found
	
	virtual bool findPathForSingleAgent(ICBSNode*  node, int ag, double lowerbound = 0) { return false; };
	virtual void  classifyConflicts(ICBSNode &parent) {};
	void findTargetConflicts(int a1, int a2, ICBSNode& curr);

	// high level search
	bool generateChild(ICBSNode* child, ICBSNode* curr);
	//conflicts
	void findConflicts(ICBSNode& curr);
	std::shared_ptr<Conflict> chooseConflict(ICBSNode &parent);
	void copyConflicts(const std::list<std::shared_ptr<Conflict>>& conflicts,
		std::list<std::shared_ptr<Conflict>>& copy, const list<int>& excluded_agent) const;
	// void copyConflicts(const std::list<std::shared_ptr<CConflict>>& conflicts,
	// 	std::list<std::shared_ptr<CConflict>>& copy, int excluded_agent) const;
	// void deleteRectConflict(ICBSNode& curr, const Conflict& conflict);
	bool hasCardinalConflict(const ICBSNode& node) const;
	bool blocked(const Path& path, const std::list<Constraint>& constraint) const;
	bool traverse(const Path& path, int loc, int t) const;
	void removeLowPriorityConflicts(std::list<std::shared_ptr<Conflict>>& conflicts) const;

	// add heuristics for the high-level search
	int computeHeuristics(const ICBSNode& curr);
	bool KVertexCover(const vector<vector<bool>>& CG, int num_of_CGnodes, int num_of_CGedges, int k);


	//update information
	// vector < list< pair<int, int> > >* collectConstraints(ICBSNode* curr, int agent_id);
	virtual void updateConstraintTable(ICBSNode* curr, int agent_id) {};
	inline void updatePaths(ICBSNode* curr);
	void updateFocalList();
	void updateReservationTable(bool* res_table, int exclude_agent, const ICBSNode &node);
	inline void releaseClosedListNodes();
	inline void releaseOpenListNodes();

	// print
	void printPaths() const;
	void printPaths(Path& path) const;
	
	void printStrategy() const;
	bool timeout=false;
};

template<class Map>
class MultiMapICBSSearch :public ICBSSearch
{
public:
    int deadline;
	boost::unordered_map<int, std::list<Constraint>> allConstraints; //Store all constraints for goal Node in online flatland
	void collectConstraints(ICBSNode* curr);

    int getBestSolutionSoFar(); // return the number of dead agents, and the paths are stored in paths
	MultiMapICBSSearch(Map * ml, AgentsLoader* al, double f_w, constraint_strategy c, int time_limit, int screen,
	        int kDlay, int deadline, options options1);
	// build MDD
	MDD<Map>* buildMDD(ICBSNode& node, int id);
	void updateConstraintTable(ICBSNode* curr, int agent_id);
	void classifyConflicts(ICBSNode &parent);
	void initializeDummyStart();
	bool isCorridorConflict(std::shared_ptr<Conflict>& corridor, const std::shared_ptr<Conflict>& con, bool cardinal, ICBSNode* node);

	bool findPathForSingleAgent(ICBSNode*  node, int ag, double lowerbound = 0);
	// Runs the algorithm until the problem is solved or time is exhausted 
	bool runICBSSearch();

	// bool updateAndReplan();
	void cleanAll();

	~MultiMapICBSSearch();
	boost::python::list outputPaths()
	{
		boost::python::list result;
		for (int i = 0; i < num_of_agents; i++)
		{
			boost::python::list agentPath;


			for (int t = 0; t < paths[i]->size(); t++) {
// 				boost::python::tuple location = boost::python::make_tuple(paths[i]->at(t).location / num_col, paths[i]->at(t).location % num_col, paths[i]->at(t).actionToHere, paths[i]->at(t).position_fraction);
				agentPath.append(paths[i]->at(t).location);
			}
			result.append(agentPath);
		}
		return result;
	}
	bool isTimeout() { return timeout; };

	bool trainCorridor1 = false;
	bool trainCorridor2 = false;

    int getSumOfHeuristicsAtStarts() const {
        int h = 0;
        for (const auto& agent : search_engines) {
            h += agent->getHeuristicAtStart();
        }
        return h;
    }

protected:
	std::vector<std::unordered_map<ConstraintsHasher, MDD<Map>*>> mddTable;
	options option;
	vector<SingleAgentICBS<Map> *> search_engines;  // used to find (single) agents' paths and mdd
	Map* ml;
	ICBSNode* goalNode;
	AgentsLoader& al;


    ConstraintTable initialConstraintTable; // store the blocked paths

    void addPathsToInitialCT(const vector<Path>& paths);
};


