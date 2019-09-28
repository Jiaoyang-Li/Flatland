#include "ecbs_search.h"


//////////////////// PRINT ///////////////////////////
template<class MyGraph>
void ECBSSearch<MyGraph>::printPaths() const
{
	/*for (int i = 0; i < num_of_agents; i++)
	{
		std::cout << "Agent " << i << " (" << paths_found_initially[i]->size() - 1 << " -->" <<
			paths[i]->size() - 1 << "): ";
		for (size_t t = 0; t < paths[i]->size(); t++)
			std::cout << "(" << paths[i]->at(t).location / num_col << "," << paths[i]->at(t).location % num_col << ")->";
		std::cout << std::endl;
	}*/
}

template<class MyGraph>
void ECBSSearch<MyGraph>::printResults() const
{
	if (runtime > time_limit)  // timeout
	{
		std::cout << "        Timeout  ; ";
	}
	else if (focal_list.empty() && solution_cost < 0)
	{
		std::cout << "No solutions  ; ";
	}
	else
	{
		std::cout << "        Succeed ; ";
	}
	std::cout << solution_cost << "," << min_sum_f_vals << "," <<
		dummy_start->sum_min_f_vals << "," <<
		prepTime << "," <<
		HL_num_expanded << "," << HL_num_generated << "," <<
		LL_num_expanded << "," << LL_num_generated << "," <<
		runtime << endl;
}

template<class MyGraph>
void ECBSSearch<MyGraph>::saveResults(const string& outputFile, const string& agentFile) const
{
	ofstream stats;
	stats.open(outputFile, std::ios::app);
	stats <<  solution_cost << "," << min_sum_f_vals <<  "," << 
		dummy_start->sum_min_f_vals << "," <<
		prepTime << "," <<
		HL_num_expanded << "," << HL_num_generated << "," <<
		LL_num_expanded << "," << LL_num_generated << "," <<
		runtime << "," <<
		disjointSplitting << "," << agentFile << "," << endl;
	stats.close();
}


//////////////////// CONSTRAINTS ///////////////////////////
// collect constraints from ancestors and build the constraint table
template<class MyGraph>
vector < list< tuple<int, int, bool> > >* ECBSSearch<MyGraph>::collectConstraints(ECBSNode* curr, int agent_id)
{
	// extract all constraints on leaf_node->agent_id
	list < Constraint > constraints_positive;
	list < Constraint > constraints_negative;

	int max_timestep = -1;
	while (curr != dummy_start)
	{
		if (get<3>(curr->constraint)) // positive constraint is valid for everyone
		{
			if (curr->agent_id == agent_id) // for the constrained agent, it is a landmark
				constraints_positive.push_back(curr->constraint);
			else // for the other agents, it is equalvelent to a negative constraint
				constraints_negative.push_back(curr->constraint);
			if (get<2>(curr->constraint) > max_timestep) // calc constraints' max_timestep
				max_timestep = get<2>(curr->constraint);
		}
		else if (curr->agent_id == agent_id)
		{
			constraints_negative.push_back(curr->constraint);
			if (get<2>(curr->constraint) > max_timestep) // calc constraints' max_timestep
				max_timestep = get<2>(curr->constraint);
		}
		curr = curr->parent;
	}


	// initialize a constraint vector of length max_timestep+1. Each entry is an empty list< tuple<int,int, bool> > (loc1,loc2, positive)
	vector < list< tuple<int, int, bool> > >* cons_vec = new vector < list< tuple<int, int, bool> > >(max_timestep + 1, list< tuple<int, int, bool> >());
	for (list< tuple<int, int, int, bool> >::iterator it = constraints_positive.begin(); it != constraints_positive.end(); it++)
	{
		if (get<1>(*it) < 0) // vertex constraint
			cons_vec->at(get<2>(*it)).push_back(make_tuple(get<0>(*it), -1, true));
		else // edge constraint
		{
			cons_vec->at(get<2>(*it) - 1).push_back(make_tuple(get<0>(*it), -1, true));
			cons_vec->at(get<2>(*it)).push_back(make_tuple(get<1>(*it), -1, true));
		}
	}
	for (list< tuple<int, int, int, bool> >::iterator it = constraints_negative.begin(); it != constraints_negative.end(); it++)
	{
		if (!get<3>(*it)) // it is a negetive constraint for this agent
		{
			cons_vec->at(get<2>(*it)).push_back(make_tuple(get<0>(*it), get<1>(*it), false));
		}
		else if (get<1>(*it) < 0) // positive vertex constraint for other agent
			cons_vec->at(get<2>(*it)).push_back(make_tuple(get<0>(*it), -1, false));
		else // positive edge constraint for other agent
		{
			cons_vec->at(get<2>(*it)).push_back(make_tuple(get<1>(*it), get<0>(*it), false));
			cons_vec->at(get<2>(*it) - 1).push_back(make_tuple(get<0>(*it), -1, false));
			cons_vec->at(get<2>(*it)).push_back(make_tuple(get<1>(*it), -1, false));
		}
	}

	return cons_vec;
}



//////////////////// TOOLS ///////////////////////////
// return agent_id's location for the given timestep
// Note -- if timestep is longer than its plan length,
// then the location remains the same as its last location
template<class MyGraph>
inline int ECBSSearch<MyGraph>::getAgentLocation(int agent_id, size_t timestep) 
{
	// if last timestep > plan length, agent remains in its last location
	if (timestep >= paths[agent_id]->size())
		return paths[agent_id]->back().location;
	else if (timestep < 0)
		return paths[agent_id]->front().location;
	else  // otherwise, return its location for that timestep
		return paths[agent_id]->at(timestep).location;
}

 // return true iff agent1 and agent2 switched locations at timestep [t,t+1] 
template<class MyGraph>
inline bool ECBSSearch<MyGraph>::switchedLocations(int agent1_id, int agent2_id, size_t timestep) {
  // if both agents at their goal, they are done moving (cannot switch places)
  if ( timestep >= paths[agent1_id]->size() && timestep >= paths[agent2_id]->size() )
    return false;
  if ( getAgentLocation(agent1_id, timestep) == getAgentLocation(agent2_id, timestep+1) &&
       getAgentLocation(agent1_id, timestep+1) == getAgentLocation(agent2_id, timestep) )
    return true;
  return false;
}

// Returns the maximal path length (among all agent)
template<class MyGraph>
int ECBSSearch<MyGraph>::getPathsMaxLength()
{
	int retVal = 0;
	for (int ag = 0; ag < num_of_agents; ag++)
		if (paths[ag] != NULL && (int)paths[ag]->size() > retVal)
			retVal = (int)paths[ag]->size();
	return retVal;
}

// adding new nodes to FOCAL (those with min-f-val*f_weight between the old and new LB)
template<class MyGraph>
void ECBSSearch<MyGraph>::updateFocalList(double old_lower_bound, double new_lower_bound, double f_weight)
{
	for (ECBSNode* n : open_list)
	{
		if (n->sum_min_f_vals > old_lower_bound && n->sum_min_f_vals <= new_lower_bound)
			n->focal_handle = focal_list.push(n);
	}
}

// takes the paths_found_initially and UPDATE all (constrained) paths found for agents from curr to start
// also, do the same for ll_min_f_vals and paths_costs (since its already "on the way").
template<class MyGraph>
inline void ECBSSearch<MyGraph>::updatePaths(ECBSNode* curr)
{
	paths = paths_found_initially;
	ll_min_f_vals = ll_min_f_vals_found_initially;
	paths_costs = paths_costs_found_initially;
	vector<bool> updated(num_of_agents, false);  // initialized for false
	while (curr->parent != NULL)
	{
		for (list<tuple<int, vector<pathEntry>, int, int>>::iterator it = curr->paths_updated.begin();
			it != curr->paths_updated.end(); it++)
		{
			if (!updated[get<0>(*it)])
			{
				paths[get<0>(*it)] = &(get<1>(*it));
				paths_costs[get<0>(*it)] = get<2>(*it);
				ll_min_f_vals[get<0>(*it)] = get<3>(*it);
				updated[get<0>(*it)] = true;
			}
		}
		curr = curr->parent;
	}
}

//////////////////// CONFLICTS ///////////////////////////
// find all conflict in the current solution
template<class MyGraph>
bool ECBSSearch<MyGraph>::finfConflicts(ECBSNode& curr)
{
	if(curr.parent == NULL) // Root node
	{
		int a2 = 0;
		for (int a1 = 0; a1 < num_of_agents; a1++) 
		{
			for (a2 = a1+1; a2 < num_of_agents; a2++) 
			{
				findConflicts(curr.conflicts, a1, a2);
			}
		}
	}
	else
	{
		// Copy from parent
		vector<bool> copy(num_of_agents, true);
		for (list<tuple<int, vector<pathEntry>, int, int>>::const_iterator it = curr.paths_updated.begin(); it != curr.paths_updated.end(); it++)
			copy[get<0>(*it)] = false;
		for (list<std::shared_ptr<Conflict>>::const_iterator it = curr.parent->conflicts.begin(); it != curr.parent->conflicts.end(); ++it)
		{
			if (copy[get<0>(**it)] && copy[get<1>(**it)])
			{
				curr.conflicts.push_back(*it);
			}
		}
		// detect new conflicts
		vector<bool> detected(num_of_agents, false);
		for (list<tuple<int, vector<pathEntry>, int, int>>::const_iterator it = curr.paths_updated.begin(); it != curr.paths_updated.end(); it++)
		{
			int a1 = get<0>(*it);
			detected[a1] = true;
			for (int a2 = 0; a2 < num_of_agents; a2++)
			{
				if (detected[a2])
					continue;
				findConflicts(curr.conflicts, a1, a2);
			}
		}
	}
	if (curr.conflicts.empty())
		return false;
	else
		return true;
}

// find conflicts between paths of agents a1 and a2
template<class MyGraph>
void ECBSSearch<MyGraph>::findConflicts(list<std::shared_ptr<Conflict>>& set, int a1, int a2)
{
	size_t min_path_length = paths[a1]->size() < paths[a2]->size() ? paths[a1]->size() : paths[a2]->size();
	for (size_t timestep = 0; timestep < min_path_length; timestep++)
	{
		int loc1 = paths[a1]->at(timestep).location;
		int loc2 = paths[a2]->at(timestep).location;
		if (loc1 == loc2)// vertex conflict
		{
			set.push_back(std::shared_ptr<Conflict>(new Conflict(a1, a2, loc1, -1, timestep))); 
		}
		else if (timestep < min_path_length - 1
			&& loc1 == paths[a2]->at(timestep + 1).location
			&& loc2 == paths[a1]->at(timestep + 1).location)// edge conflict
		{
			set.push_back(std::shared_ptr<Conflict>(new Conflict(a1, a2, loc1, loc2, timestep + 1))); 
		}
	}
	if (paths[a1]->size() != paths[a2]->size()) 
	{// check whether there are conflicts that occur after one agent reaches its goal
		int a1_ = paths[a1]->size() < paths[a2]->size() ? a1 : a2;
		int a2_ = paths[a1]->size() < paths[a2]->size() ? a2 : a1;
		int loc1 = paths[a1_]->back().location;
		for (size_t timestep = min_path_length; timestep < paths[a2_]->size(); timestep++)
		{
			int loc2 = paths[a2_]->at(timestep).location;
			if (loc1 == loc2)
			{// It's at least a semi conflict			
				set.push_front(std::shared_ptr<Conflict>(new Conflict(a1_, a2_, loc1, -1, timestep)));
			}
		}
	}
}

// choose a conflict to split
template<class MyGraph>
std::shared_ptr<Conflict> ECBSSearch<MyGraph>::chooseConflict(const list<std::shared_ptr<Conflict>>& confs)
{
	int id = rand() % confs.size();
	int i = 0;
	for (auto it: confs)
	{
		if (i == id)
			return it;
		else
			i++;
	}
	return NULL;
}

// Generates a boolean reservation table (i.e., conflict avoidance table) for paths (cube of map_size*max_timestep).
// This is used by the low-level ECBS to count possible collisions efficiently
// Note -- we do not include the agent for which we are about to plan for
template<class MyGraph>
void ECBSSearch<MyGraph>::updateReservationTable(size_t max_path_len, int exclude_agent)
{
	cat.clear();
	cat.resize(max_path_len);
	for (int ag = 0; ag < num_of_agents; ag++) 
	{
		if (ag != exclude_agent && paths[ag] != NULL)
		{
			for (size_t timestep = 0; timestep < max_path_len; timestep++)
			{
				int id = getAgentLocation(ag, timestep);
				cat[timestep].insert(id);
				int prev_id = getAgentLocation(ag, timestep - 1);
				if (prev_id != id)
				{
					cat[timestep].insert(id + (1 + prev_id) * (int)map_size);
				}
			}
		}
	}
}



//////////////////// GENERATE A NODE ///////////////////////////
// plan a path for an agent in a child node
template<class MyGraph>
bool ECBSSearch<MyGraph>::findPathForSingleAgent(ECBSNode*  node, int ag)
{
	// extract all constraints on agent ag
	ECBSNode* curr = node;
	vector < list< tuple<int, int, bool> > >* cons_vec = collectConstraints(curr, ag);
	// build reservation table
	int max_plan_len = getPathsMaxLength();
	updateReservationTable(max_plan_len, ag);
	// find a path w.r.t cons_vec (and prioretize by res_table).
	bool foundSol = single_planner.runFocalSearch(G, ag, focal_w, cons_vec, cat);
	LL_num_expanded += single_planner.num_expanded;
	LL_num_generated += single_planner.num_generated;
	delete (cons_vec);
	if (foundSol)
	{
		node->paths_updated.emplace_back(ag, vector<pathEntry>(single_planner.path),
			single_planner.path_cost, single_planner.min_f_val);
		node->g_val = node->g_val - (int)paths[ag]->size() + (int)single_planner.path.size();
		node->sum_min_f_vals = node->sum_min_f_vals - ll_min_f_vals[ag] + single_planner.min_f_val;
		paths[ag] = &get<1>(node->paths_updated.back());
		ll_min_f_vals[ag] = single_planner.min_f_val;
		return true;
	}
	else
	{
		delete node;
		return false;
	}
}

// plan paths for a child node
template<class MyGraph>
bool ECBSSearch<MyGraph>::generateChild(ECBSNode*  node)
{
	if (get<3>(node->constraint)) //positve constraint
	{
		for (int ag = 0; ag < num_of_agents; ag++)
		{
			if (ag == node->agent_id)
				continue;
			else if (get<1>(node->constraint) < 0 && // vertex constraint
				getAgentLocation(ag, get<2>(node->constraint)) == get<0>(node->constraint))
			{
				if (!findPathForSingleAgent(node, ag))
					return false;
			}
			else if (get<1>(node->constraint) >= 0 && //edge constraint
				getAgentLocation(ag, get<2>(node->constraint) - 1) == get<1>(node->constraint) &&
				getAgentLocation(ag, get<2>(node->constraint)) == get<0>(node->constraint))
			{
				if (!findPathForSingleAgent(node, ag))
					return false;
			}
		}
	}
	else // negative constraint
	{
		if (!findPathForSingleAgent(node, node->agent_id))
			return false;
	}

	finfConflicts(*node);
	node->num_of_collisions = (int)node->conflicts.size(); //computeNumOfCollidingAgents();

	// update handles
	node->open_handle = open_list.push(node);
	HL_num_generated++;
	node->time_generated = HL_num_generated;
	if (node->sum_min_f_vals <= focal_list_threshold)
		node->focal_handle = focal_list.push(node);
	//allNodes_table[n1] = n1;
	allNodes_table.push_back(node);
	return true;
}


//////////////////// MAIN FUNCTIONS ///////////////////////////
// RUN ECBS
template<class MyGraph>
bool ECBSSearch<MyGraph>::runECBSSearch()
{
	std::cout << "ECBS: " << std::endl;
	// set timer
	std::clock_t start;
	start = std::clock();

	// start is already in the open_list
	while (!focal_list.empty())
	{
		runtime = (std::clock() - start) * 1000.0 / CLOCKS_PER_SEC;
		if (runtime > time_limit) // timeout
		{
			break;
		}

		ECBSNode* curr = focal_list.top();
		focal_list.pop();
		open_list.erase(curr->open_handle);

		// takes the paths_found_initially and UPDATE all constrained paths found for agents from curr to dummy_start (and lower-bounds)
		updatePaths(curr);

		if (curr->conflicts.empty())
		{  // found a solution (and finish the while look)
			solution_found = true;
			solution_cost = curr->g_val;
			break;
		}

		// generate the two successors that resolve one of the conflicts
		HL_num_expanded++;
		curr->time_expanded = HL_num_expanded;
		int agent1_id, agent2_id, location1, location2, timestep;
		tie(agent1_id, agent2_id, location1, location2, timestep) = *chooseConflict(curr->conflicts);

		ECBSNode* n1 = new ECBSNode();
		ECBSNode* n2 = new ECBSNode();
		n1->parent = curr;
		n2->parent = curr;
		n1->g_val = curr->g_val;
		n2->g_val = curr->g_val;
		n1->sum_min_f_vals = curr->sum_min_f_vals;
		n2->sum_min_f_vals = curr->sum_min_f_vals;

		if (disjointSplitting)
		{
			int id;
			if (rand() % 2 == 0)
				id = agent1_id;
			else
				id = agent2_id;

			n1->agent_id = id;
			n2->agent_id = id;
			if (location2 >= 0 && id == agent2_id) //edge constraint for the second agent
			{
				n1->constraint = make_tuple(location2, location1, timestep, true);
				n2->constraint = make_tuple(location2, location1, timestep, false);
			}
			else
			{
				n1->constraint = make_tuple(location1, location2, timestep, true);
				n2->constraint = make_tuple(location1, location2, timestep, false);
			}
		}
		else //Non-disjoint
		{
			n1->agent_id = agent1_id;
			n2->agent_id = agent2_id;
			n1->constraint = make_tuple(location1, location2, timestep, false);
			if (location2 < 0)
				n2->constraint = make_tuple(location1, location2, timestep, false);
			else
				n2->constraint = make_tuple(location2, location1, timestep, false);
		}
		vector<vector<pathEntry>*> copy(paths);
		generateChild(n1);
		paths = copy;
		generateChild(n2);
		curr->conflicts.clear();
		if (open_list.size() == 0)
		{
			solution_found = false;
			break;
		}
		ECBSNode* open_head = open_list.top();
		if (open_head->sum_min_f_vals > min_sum_f_vals)
		{
			min_sum_f_vals = open_head->sum_min_f_vals;
			double new_focal_list_threshold = min_sum_f_vals * focal_w;
			updateFocalList(focal_list_threshold, new_focal_list_threshold, focal_w);
			focal_list_threshold = new_focal_list_threshold;
		}

	}  // end of while loop


	runtime = (std::clock() - start) * 1000.0 / CLOCKS_PER_SEC; //  get time
	printResults();
	return solution_found;
}

template<class MyGraph>
ECBSSearch<MyGraph>::ECBSSearch(const MyGraph& G, double focal_w, bool disjointSplitting, int cutoffTime) :
	focal_w(focal_w), disjointSplitting(disjointSplitting), G(G), single_planner(G.map_size())
{
	// set timer
	std::clock_t start;
	start = std::clock();

	time_limit = cutoffTime * 1000;
	num_of_agents = (int)G.start_locations.size();
	map_size = G.map_size();
	ll_min_f_vals = vector <int>(num_of_agents);
	paths_costs = vector <int>(num_of_agents);
	ll_min_f_vals_found_initially = vector <int>(num_of_agents);
	paths_costs_found_initially = vector <int>(num_of_agents);

	// initialize paths_found_initially
	paths_found_initially.resize(num_of_agents, NULL);
	for (int i = 0; i < num_of_agents; i++)
	{
		paths = paths_found_initially;
		int max_plan_len = getPathsMaxLength();
		updateReservationTable(max_plan_len, i);
		if (single_planner.runFocalSearch(G, i, focal_w, NULL, cat) == false)
		{
			cout << "NO SOLUTION EXISTS FOR AGENT " << i;
			exit(-1);
		}
		paths_found_initially[i] = new vector<pathEntry>(single_planner.path);
		ll_min_f_vals_found_initially[i] = single_planner.min_f_val;
		paths_costs_found_initially[i] = single_planner.path_cost;
		LL_num_expanded += single_planner.num_expanded;
		LL_num_generated += single_planner.num_generated;
	}

	paths = paths_found_initially;
	ll_min_f_vals = ll_min_f_vals_found_initially;
	paths_costs = paths_costs_found_initially;

	// generate dummy start and update data structures
	dummy_start = new ECBSNode();
	dummy_start->agent_id = -1;
	dummy_start->g_val = 0;
	dummy_start->sum_min_f_vals = 0;
	for (int i = 0; i < num_of_agents; i++)
	{
		dummy_start->g_val += paths_costs[i];
		dummy_start->sum_min_f_vals += ll_min_f_vals[i];
	}
	finfConflicts(*dummy_start);
	dummy_start->num_of_collisions = (int)dummy_start->conflicts.size();
	dummy_start->open_handle = open_list.push(dummy_start);
	dummy_start->focal_handle = focal_list.push(dummy_start);
	HL_num_generated++;
	dummy_start->time_generated = HL_num_generated;

	allNodes_table.push_back(dummy_start);

	min_sum_f_vals = dummy_start->sum_min_f_vals;
	focal_list_threshold = focal_w * dummy_start->sum_min_f_vals;

	prepTime = std::clock() - start;
}

template<class MyGraph>
ECBSSearch<MyGraph>::~ECBSSearch()
{
	for (size_t i = 0; i < paths_found_initially.size(); i++)
		delete (paths_found_initially[i]);
	for (list<ECBSNode*>::iterator it = allNodes_table.begin(); it != allNodes_table.end(); it++)
		delete *it;
}
