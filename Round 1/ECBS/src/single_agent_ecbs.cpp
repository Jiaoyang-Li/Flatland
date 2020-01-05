#include "single_agent_ecbs.h"


void SingleAgentPlanner::clear()
{
	this->num_expanded = 0;
	this->num_generated = 0;
	this->path_cost = 0;
	this->lower_bound = 0;
	this->min_f_val = 0;
	open_list.clear();
	focal_list.clear();
	allNodes_table.clear();
}


// Updates the path.
void SingleAgentPlanner::updatePath(Node* goal)
{
	path.clear();
	Node* curr = goal;
	while ( curr != nullptr)
	{
		path.emplace_back(curr->id, curr->timestep);
		curr = curr->parent;
	}
	reverse(path.begin(), path.end());
	path_cost = goal->g_val;
	num_of_conf = goal->num_internal_conf;
}

void SingleAgentPlanner::releaseClosedListNodes(hashtable_t& allNodes_table)
{
	hashtable_t::iterator it;
	for (it=allNodes_table.begin(); it != allNodes_table.end(); it++) 
	{
		delete *it; 
	}
}


// iterate over the constraints ( cons[t] is a list of all constraints for timestep t) and return the latest
// timestep which has a constraint involving the goal location
// which is the minimal plan length for the agent
int SingleAgentPlanner::extractLastGoalTimestep(int goal_location, const vector< list< tuple<int, int, bool> > >* cons)
{
	if (cons == NULL)
		return -1;
	for (int t = (int)cons->size() - 1; t > 0; t--)
	{
		for (auto it = cons->at(t).begin(); it != cons->at(t).end(); ++it)
		{
			if (get<0>(*it) == goal_location && get<1>(*it) < 0 && !get<2>(*it)) 
			{
				return (t);
			}
		}
	}
	return -1;
}

// Checks if a vaild path found (wrt my_map and constraints)
// input: curr_id (location at time next_timestep-1) ; next_id (location at time next_timestep); next_timestep
// cons[timestep] is a list of <loc1,loc2, bool> of (vertex/edge) constraints for that timestep. (loc2=-1 for vertex constraint).
bool SingleAgentPlanner::isConstrained(int next_loc, int curr_timestesp, int next_timestep, const vector< list< int > >& cons)  const
{
	if (cons.empty())
		return false;
	if (next_loc < 0)
		return false;
	// check vertex constraints (being in next_id at next_timestep is disallowed)
	for (int t = curr_timestesp; t <= min(next_timestep, (int)cons.size() - 1); t++)
	{
		for (auto loc : cons[t])
		{
			if (loc == next_loc) // edge constraint
				return true;
		}
	}
	return false;
}

// Return the number of conflicts between the known_paths' (by looking at the reservation table) for the move [curr_id,next_id].
// Returns 0 if no conflict, 1 for vertex or edge conflict, 2 for both.
int SingleAgentPlanner::numOfConflictsForStep(int curr_loc, int next_loc, int next_timestep, const CAT& res_table, size_t map_size)
{
	if (next_timestep >= (int)res_table.size())
		return 0;

	int retVal = 0;
	// check vertex constraints (being in next_id at next_timestep is disallowed)
	if (next_loc >= 0 &&  res_table[next_timestep].find(next_loc) != res_table[next_timestep].end())
		retVal++;
	// check edge constraints (the move from curr_id to next_id at next_timestep-1 is disallowed)
	// which means that res_table is occupied with another agent for [curr_id,next_timestep] and [next_id,next_timestep-1]
	if (curr_loc >= 0 && next_loc >= 0 && res_table[next_timestep].find(curr_loc + (1 + next_loc) *(int)map_size) != res_table[next_timestep].end())
		retVal++;

	return retVal;
}

// Iterate over OPEN and adds to FOCAL all nodes with: 
// 1) f-val > old_min_f_val ; and 
// 2) f-val * f_weight < new_lower_bound.
void SingleAgentPlanner::updateFocalList(double old_lower_bound, double new_lower_bound, double f_weight)
{
	for (Node* n : open_list)
	{
		if ( n->getFVal() > old_lower_bound && n->getFVal() <= new_lower_bound )
		{
			n->focal_handle = focal_list.push(n);
		}
	}
}



