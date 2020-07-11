#include "ConstraintTable.h"
void ConstraintTable::insert(int loc, int t_min, int t_max)
{
    assert(loc >= 0);
    if (CT[loc].empty())
        CT[loc].resize(length_max + 1, false);
	if (t_max == 0) {
        //CT[loc].emplace(t_min);
        CT[loc][t_min] = true;
	}
	else {
	    for (int t = t_min; t < t_max; t++)
            //CT[loc].emplace(t);
            CT[loc][t] = true;
	}
}

void ConstraintTable::insert_to_fixed_CT(int loc, int timestep, int kRobust)
{
    assert(loc >= 0);
    if (CT_paths[loc].empty())
        CT_paths[loc].resize(length_max + 1, false);
    for (int t = timestep - kRobust; t <= timestep; t++)
        CT_paths[loc][t] = true;
}


bool ConstraintTable::is_constrained(int loc, int timestep, int kRobust) const
{
    if (loc < 0)
        return false;
    //if (CT[loc].count(timestep))
    if (!CT[loc].empty() && CT[loc][timestep])
        return true;

    if (CT_paths[loc].empty())
        return false;
    for (int t = timestep - kRobust; t <= timestep; t++)
    {
        if (CT_paths[loc][t])
            return true;
    }
	return false;
}

/*bool ConstraintTable::is_good_malfunction_location(int loc, int t)
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
}*/

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