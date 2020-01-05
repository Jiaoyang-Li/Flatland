#pragma once
#include "node.h"

class SingleAgentPlanner 
{
private:
	// define typedefs (will also be used in ecbs_search)
	// note -- handle typedefs is defined inside the class (hence, include node.h is not enough).
	typedef fibonacci_heap< Node*, compare<Node::compare_node> > heap_open_t;
	typedef fibonacci_heap< Node*, compare<Node::secondary_compare_node> > heap_focal_t;
	typedef unordered_set<Node*, Node::NodeHasher, Node::eqnode> hashtable_t;
	heap_open_t open_list;
	heap_focal_t focal_list;
	hashtable_t allNodes_table;

	void clear();
	int extractLastGoalTimestep(int goal_location, const vector< list< tuple<int, int, bool> > >* cons);
	void releaseClosedListNodes(hashtable_t& allNodes_table);
	bool isConstrained(int next_id, int curr_timestesp, int next_timestep, const vector< list< int > >& cons) const;
	void updatePath(Node* goal);
	int numOfConflictsForStep(int curr_id, int next_id, int next_timestep, const CAT& res_table, size_t map_size);
	void updateFocalList(double old_lower_bound, double new_lower_bound, double f_weight);

public:
	SingleAgentPlanner(size_t map_size, int max_makespan): map_size(map_size), max_makespan(max_makespan) {}

	vector<pathEntry> path;  // a path that takes the agent from initial to goal location satisying all constraints
	int path_cost;  
	double lower_bound;  // FOCAL's lower bound ( = e_weight * min_f_val)
	int min_f_val;  // min f-val seen so far
	int num_of_conf;

	const size_t map_size;
	const int max_makespan;

	uint64_t num_expanded;
	uint64_t num_generated;

	// Returns true if a collision free path found (with cost up to f_weight * f-min) while
	// minimizing the number of internal conflicts (that is conflicts with known_paths for other agents found so far).
	template<class MyGraph>
	void runFocalSearch(const MyGraph& G, int agent_id, double f_weight,
		const vector < list< int > >& constraints,
		const CAT& res_table)
	{
		clear();
		hashtable_t::iterator it;  // will be used for find()

		// generate start and add it to the OPEN list
		int start_id = G.start_ids[agent_id];
		int initial_h = G.get_h_value(agent_id, start_id);
		if (initial_h >= min(max_makespan, (int)G.map_size() * G.r_velocities[agent_id])) // start and goal locations are disconnected
		{
			path.clear();
			min_f_val = max_makespan + 1;
			path_cost = max_makespan + 1;
			return;
		}
		Node* start = new Node(-1, 0, initial_h + 1, nullptr, 0, false);
		num_generated++;

		start->open_handle = open_list.push(start);
		start->focal_handle = focal_list.push(start);
		start->in_openlist = true;
		allNodes_table.insert(start);
		min_f_val = start->getFVal();
		lower_bound = f_weight * min_f_val; // max(lowerbound, f_weight * min_f_val);

		// int lastGoalConsTime = extractLastGoalTimestep(G.goal_locations[agent_id], constraints);

		while (!focal_list.empty())
		{
			Node* curr = focal_list.top(); focal_list.pop();
			open_list.erase(curr->open_handle);
			curr->in_openlist = false;
			num_expanded++;

			// check if the popped node is a goal
			if (G.get_location(curr->id) == G.goal_locations[agent_id]) // && curr->timestep > lastGoalConsTime)
			{
				updatePath(curr);
				releaseClosedListNodes(allNodes_table);
				return;
			}

			// iterator over all possible actions
			list<int> neighbours;
			if (curr->id >= 0)
				neighbours = G.children_vertices(curr->id);
			else
			{
				neighbours.emplace_back(-1); // wait at the station
				neighbours.emplace_back(start_id); // move to the start location
			}
			for (auto next_id : neighbours)
			{
				int next_timestep, next_h_val;
				if (next_id >= 0)
				{
					if (curr->id == next_id || curr->id < 0)
						next_timestep = curr->timestep + 1;
					else
						next_timestep = curr->timestep + G.r_velocities[agent_id];
					next_h_val = G.get_h_value(agent_id, next_id);
				}
				else
				{
					next_timestep = curr->timestep + 1;
					next_h_val = initial_h + 1;
				}
				int next_g_val = next_timestep;
				if (next_h_val > max_makespan - next_g_val) //we cannot reach the goal location before the ddl
					continue;
				if (!isConstrained(G.get_location(next_id), curr->timestep, next_timestep, constraints)) // TODO: this method does not work for edge constraints
				{
					int next_internal_conflicts = curr->num_internal_conf + numOfConflictsForStep(G.get_location(curr->id),
						G.get_location(next_id), next_timestep, res_table, G.map_size());
					// generate (maybe temporary) node
					Node* next = new Node(next_id, next_g_val, next_h_val,
						curr, next_timestep, next_internal_conflicts, false);
					// try to retrieve it from the hash table
					it = allNodes_table.find(next);

					if (it == allNodes_table.end())
					{
						next->open_handle = open_list.push(next);
						next->in_openlist = true;
						num_generated++;

						if (next->getFVal() <= lower_bound)
							next->focal_handle = focal_list.push(next);
						allNodes_table.insert(next);
					}
					else
					{  // update existing node's if needed (only in the open_list)
						delete(next);  // not needed anymore -- we already generated it before
						Node* existing_next = *it;
						if (existing_next->in_openlist)
						{  // if its in the open list
							if (existing_next->getFVal() > next_g_val + next_h_val ||
								(existing_next->getFVal() == next_g_val + next_h_val &&
									existing_next->num_internal_conf > next_internal_conflicts))
							{
								// if f-val decreased through this new path (or it remains the same and there's less internal conflicts)
								bool add_to_focal = false;  // check if it was above the focal bound before and now below (thus need to be inserted)
								bool update_in_focal = false;  // check if it was inside the focal and needs to be updated (because f-val changed)
								bool update_open = false;
								if ((next_g_val + next_h_val) <= lower_bound)
								{  // if the new f-val qualify to be in FOCAL
									if (existing_next->getFVal() > lower_bound)
										add_to_focal = true;  // and the previous f-val did not qualify to be in FOCAL then add
									else
										update_in_focal = true;  // and the previous f-val did qualify to be in FOCAL then update
								}
								if (existing_next->getFVal() > next_g_val + next_h_val)
									update_open = true;
								// update existing node
								existing_next->g_val = next_g_val;
								existing_next->h_val = next_h_val;
								existing_next->parent = curr;
								existing_next->num_internal_conf = next_internal_conflicts;

								if (update_open)
								{
									open_list.increase(existing_next->open_handle);  // increase because f-val improved
								}
								if (add_to_focal)
								{
									existing_next->focal_handle = focal_list.push(existing_next);
								}
								if (update_in_focal)
								{
									focal_list.update(existing_next->focal_handle);
								}
							}
						}
						else
						{  // if its in the closed list (reopen)
							if (existing_next->getFVal() > next_g_val + next_h_val ||
								(existing_next->getFVal() == next_g_val + next_h_val 	&&
									existing_next->num_internal_conf > next_internal_conflicts))
							{
								// if f-val decreased through this new path (or it remains the same and there's less internal conflicts)
								existing_next->g_val = next_g_val;
								existing_next->h_val = next_h_val;
								existing_next->parent = curr;
								existing_next->num_internal_conf = next_internal_conflicts;
								existing_next->open_handle = open_list.push(existing_next);
								existing_next->in_openlist = true;
								if (existing_next->getFVal() <= lower_bound)
								{
									existing_next->focal_handle = focal_list.push(existing_next);
								}
							}
						}  // end update a node in closed list
					}  // end update an existing node
				}
			}  // end for loop that generates successors

			if (open_list.empty())  // in case OPEN is empty, no path found...
				break;

			// update FOCAL if min f-val increased
			auto open_head = open_list.top();
			if (open_head->getFVal() > min_f_val)
			{
				int new_min_f_val = open_head->getFVal();
				double new_lower_bound = f_weight * new_min_f_val; // max(lowerbound, f_weight * new_min_f_val);
				updateFocalList(lower_bound, new_lower_bound, f_weight);
				min_f_val = new_min_f_val;
				lower_bound = new_lower_bound;
			}
		}  // end while loop
		   // no path found
		path.clear();
		min_f_val = max_makespan + 1;
		path_cost = max_makespan + 1;
		releaseClosedListNodes(allNodes_table);
		return;
	}
};


