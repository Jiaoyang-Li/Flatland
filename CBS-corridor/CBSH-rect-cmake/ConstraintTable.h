#pragma once
#include "Conflict.h"
#include <climits>
#include <unordered_set>
class ConstraintTable
{
public:
	int length_min = 0;
	int length_max = INT_MAX;
	int goal_location = -1;
	int latest_timestep = 0; // No negative constraints after this timestep.

	void copy(const ConstraintTable& other)
    {
        length_min = other.length_min;
        length_max = other.length_max;
        goal_location = other.goal_location;
        latest_timestep = other.latest_timestep;
        CT_Single = other.CT_Single;
        CT = other.CT;
    }
	void clear(){
		CT.clear(); 
		length_min = 0, 
		length_max = INT_MAX; 
		latest_timestep = 0;
		CT_Single.clear();
	}
	void insert(int loc, int t_min, int t_max);
	bool is_constrained(int loc, int t);
	void printSize() {
		std::cout << CT.size() << std::endl;;
	};
	bool is_good_malfunction_location(int loc, int t);

private:
	unordered_map<size_t, std::unordered_set<int> > CT_Single;
    unordered_map<size_t, list<pair<int, int> > > CT;
};

