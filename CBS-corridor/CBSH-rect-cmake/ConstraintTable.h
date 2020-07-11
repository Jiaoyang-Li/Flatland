#pragma once
#include "Conflict.h"
#include <climits>
#include <unordered_set>
#include <set>
class ConstraintTable
{
public:
	int length_max = INT_MAX;

	void clear(){
		CT.clear();
		CT.resize(CT_paths.size());
	}
	void insert(int loc, int t_min, int t_max); // insert a constraint
	void insert_to_fixed_CT(int loc, int t, int kRobust = 1);
	bool is_constrained(int loc, int t, int kRobust = 1) const;
	// bool is_good_malfunction_location(int loc, int t);

    void init(size_t map_size)
    {
        CT_paths.resize(map_size);
        CT.resize(map_size);
    }
private:
    vector< vector<bool> > CT; // this stores the constraints from CBS
    vector< vector<bool> > CT_paths; // this stores the already planned paths
};

