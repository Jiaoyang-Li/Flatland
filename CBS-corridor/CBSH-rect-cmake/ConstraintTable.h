#pragma once
#include "Conflict.h"
#include <climits>
#include <unordered_set>
#include <set>
class ConstraintTable
{
public:
	int length_max = INT_MAX;

	void clear()
	{
		CT.clear();
		CT.resize(CT_paths.size());
	}

	void insert(int loc, int t_min, int t_max); // insert a constraint
	bool insert_path(int agent_id, const Path& path, int kRobust = 1);
    void delete_path(int agent_id, const Path& path, int kRobust = 1);
	bool is_constrained(int loc, int t, int kRobust = 1) const;
	// bool is_good_malfunction_location(int loc, int t);
	void get_conflicting_agents(set<int>& conflicting_agents, int loc, int t, int kRobust = 1) const;

    void init(size_t map_size)
    {
        CT_paths.resize(map_size);
    }
private:
    vector< vector<bool> > CT; // this stores the constraints from CBS
    vector< vector<int> > CT_paths; // this stores the already planned paths, the value is the id of the agent
};

