#include "ConstraintTable.h"
void ConstraintTable::insert(int loc, int t_min, int t_max)
{
    assert(loc >= 0);
	if (t_max == 0) {
		//if (t_min <= length_min) {
			CT_Single[loc].emplace(t_min);
			t_max = t_min + 1;
		//}
		//else {
		//	CT[loc].emplace_back(length_min,t_min+1);
		//}
	}
	else {
	    for (int t = t_min; t < t_max; t++)
            CT_Single[loc].emplace(t);
		//CT[loc].emplace_back(t_min, t_max);
	}
	
	if (loc == goal_location && t_max > length_min)
	{
		length_min = t_max;
	}
	if (t_max < INT_MAX && t_max > latest_timestep)
	{
		latest_timestep = t_max;
	}
}

bool ConstraintTable::is_constrained(int loc, int t)
{
    if (loc < 0)
        return false;
	//if (CT_Single.count(loc)) {
		if (CT_Single[loc].count(t)) {
			return true;
		}
	//}
	auto it = CT.find(loc);
	if (it == CT.end())
	{
		return false;
	}
	for (auto constraint : it->second)
	{
		if (constraint.first <= t && t < constraint.second)
			return true;
	}
	return false;
}

bool ConstraintTable::is_good_malfunction_location(int loc, int t)
{
    if (loc <0)
        return true;
	//if (CT_Single.count(loc)) {
		for (auto conT : CT_Single[loc]) {
			if (conT >= t) {
				return false;
			}
		}

	//}
	auto it = CT.find(loc);
	if (it == CT.end())
	{
		return true;
	}
	for (auto constraint : it->second)
	{
		if (constraint.first >= t)
			return false;
	}
	return true;
}

//void ConstraintTable::insert(int loc, int t_min, int t_max)
//{
//
//	for (int t = t_min; t < t_max; t++) {
//		CT[loc].insert(t);
//	}
//	if (loc == goal_location && t_max > length_min)
//	{
//		length_min = t_max;
//	}
//	if (t_max < INT_MAX && t_max > latest_timestep)
//	{
//		latest_timestep = t_max;
//	}
//}
//
//bool ConstraintTable::is_constrained(int loc, int t)
//{
//	auto it = CT.find(loc);
//	if (it == CT.end())
//	{
//		return false;
//	}
//	
//	if (CT[loc].count(t))
//		return true;
//	else
//		return false;
//}