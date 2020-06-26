#include "ecbs_search.h"

#include <memory>

template<class MyGraph>
bool ECBSSearch<MyGraph>::evaluateSolution() const
{
    for (size_t i = 0; i < paths.size(); i++)
    {
		if (paths[i]->empty())
			continue;
        for (size_t timestep = 0; timestep < paths[i]->size() - 1; timestep++)
        {
            int id = paths[i]->at(timestep).id;
            int next_id = paths[i]->at(timestep + 1).id;
            bool connected = false;
			if (id < 0)
			{
				continue;
			}
            for (auto children : G.children_vertices(id))
            {
                if (children == next_id)
                {
                    connected = true;
                    break;
                }
            }
            if (!connected)
                return false;
        }
    }
    for (size_t i = 0; i <  paths.size(); i++)
    {
        for (size_t j = i + 1; j <  paths.size(); j++)
        {
            auto conflict = findConflicts(i, j);
			if (conflict != nullptr)
			{
				cout << "Found a conflict " << *conflict << endl;
				return false;
			}
        }
    }
    return true;
}

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
	cout << num_of_agents << " agents -- ";
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
	std::cout << "makespan: " << solution_makespan << " ;sum of cost: " << solution_cost << " ;lowerbound: " << min_sum_f_vals << //"," <<
		// dummy_start->sum_min_f_vals << "," <<
        "; runtime: " << runtime <<
        "; nodes:   " << HL_num_expanded << //"," << HL_num_generated << "," <<
		//LL_num_expanded << "," << LL_num_generated << "," <<
		endl;
	if (solution_cost >= 0)
	{
		int count = 0;
		for (auto const& path : paths)
		{
			if (path->empty())
				count++;
		}
		if (count > 0)
			cout << "Give up " << count << " agents!" << endl;
	}
}

template<class MyGraph>
void ECBSSearch<MyGraph>::saveResults(const string& outputFile, const string& agentFile) const
{
	ofstream stats;
	stats.open(outputFile, std::ios::app);
	stats <<  solution_cost << "," << min_sum_f_vals <<  "," << 
		dummy_start->sum_min_f_vals << "," <<
		HL_num_expanded << "," << HL_num_generated << "," <<
		LL_num_expanded << "," << LL_num_generated << "," <<
		runtime << "," <<
		disjointSplitting << "," << agentFile << "," << endl;
	stats.close();
}


//////////////////// CONSTRAINTS ///////////////////////////
// collect constraints from ancestors and build the constraint table
template<class MyGraph>
void ECBSSearch<MyGraph>::collectConstraints(ECBSNode* curr, int agent_id, vector < list< int > >& cons_vec)
{
	// extract all constraints on leaf_node->agent_id
	// list < Constraint > constraints_positive;
	list < Constraint > constraints_negative;

	int max_timestep = -1;
	while (curr != dummy_start)
	{
		/*if (get<3>(curr->constraint)) // positive constraint is valid for everyone
		{
			if (curr->agent_id == agent_id) // for the constrained agent, it is a landmark
				constraints_positive.push_back(curr->constraint);
			else // for the other agents, it is equalvelent to a negative constraint
				constraints_negative.push_back(curr->constraint);
			if (get<2>(curr->constraint) > max_timestep) // calc constraints' max_timestep
				max_timestep = get<2>(curr->constraint);
		}
		else*/
		if (curr->agent_id == agent_id)
		{
			constraints_negative.push_back(curr->constraint);
			if (get<3>(curr->constraint) > max_timestep) // calc constraints' max_timestep
				max_timestep = get<3>(curr->constraint);
		}
		curr = curr->parent;
	}

	for (const auto& path : G.paths)
	{
		if (!path.empty())
			max_timestep = max(max_timestep, path.back().timestep);
	}

	// initialize a constraint vector of length max_timestep+1. Each entry is an empty list< tuple<int,int, bool> > (loc1,loc2, positive)
	cons_vec.resize(max_timestep + 1);
	/*for (const auto & it : constraints_positive)
	{
		if (get<1>(it) < 0) // vertex constraint
			cons_vec->at(get<2>(it)).push_back(make_tuple(get<0>(it), -1, true));
		else // edge constraint
		{
			cons_vec->at(get<2>(it) - 1).push_back(make_tuple(get<0>(it), -1, true));
			cons_vec->at(get<2>(it)).push_back(make_tuple(get<1>(it), -1, true));
		}
	}*/
	for (const auto & it : constraints_negative)
	{
		//if (!get<3>(it)) // it is a negetive constraint for this agent
		//{
		    for (int t = get<2>(it); t < get<3>(it); t++)
			    cons_vec[t].push_back(get<0>(it));
		/*}
		else if (get<1>(it) < 0) // positive vertex constraint for other agent
        {
            for (int t = get<2>(*it) - ; t <= get<3>(it); t++)
                cons_vec->at(t).push_back(make_tuple(get<0>(it), get<1>(it), false));
        }	cons_vec->at(get<2>(it)).push_back(make_tuple(get<0>(it), -1, false));
		else // positive edge constraint for other agent
		{
			cons_vec->at(get<2>(*it)).push_back(make_tuple(get<1>(*it), get<0>(*it), false));
			cons_vec->at(get<2>(*it) - 1).push_back(make_tuple(get<0>(*it), -1, false));
			cons_vec->at(get<2>(*it)).push_back(make_tuple(get<1>(*it), -1, false));
		}*/
	}

	for (const auto& path : G.paths)
	{
		if (path.empty())
			continue;
		for (int step = 1; step < (int)path.size() - 1; step++)
		{
			if (path[step].id >= 0)
			{
				int loc = G.get_location(path[step].id);
				for (int t = path[step].timestep; t <= path[step + 1].timestep; t++)
				{
					cons_vec[t].push_back(loc);
				}
				int next_loc = G.get_location(path[step + 1].id);
				if (loc != next_loc)
				{
					for (int t = path[step].timestep; t <= path[step + 1].timestep; t++)
					{
						cons_vec[t].push_back(next_loc);
					}
				}
			}
		}
		// int goal = G.get_location(path.back().id);
		// cons_vec[path.back().timestep].push_back(goal);
	}
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
	while (curr->parent != nullptr)
	{
		for (auto it = curr->paths_updated.begin();
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
bool ECBSSearch<MyGraph>::findConflicts(ECBSNode& curr)
{
	if(curr.parent == nullptr) // Root node
	{
		int a2 = 0;
		for (int a1 = 0; a1 < num_of_agents; a1++) 
		{
			for (a2 = a1+1; a2 < num_of_agents; a2++) 
			{
				auto conflict = findConflicts(a1, a2);
				if (conflict != nullptr)
					curr.conflicts.push_back(conflict);
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
				auto conflict = findConflicts(a1, a2);
				if (conflict != nullptr)
					curr.conflicts.push_back(conflict);
			}
		}
	}
    return !curr.conflicts.empty();
}

// find conflicts between paths of agents a1 and a2
template<class MyGraph>
std::shared_ptr<Conflict> ECBSSearch<MyGraph>::findConflicts(int a1, int a2) const
{
	if (paths[a1]->empty() || paths[a2]->empty())
		return nullptr;
	Path::const_iterator state1 = paths[a1]->begin();
	Path::const_iterator state2 = paths[a2]->begin();
	while (state1 != paths[a1]->end() && state2 != paths[a2]->end())
	{
		if (state1->id < 0)
		{
			++state1;
			continue;
		}
		if (state2->id < 0)
		{
			++state2;
			continue;
		}

		int loc1 = G.get_location(state1->id);
		int loc2 = G.get_location(state2->id);

		auto next1 = state1;
		++next1;
		if (loc1 == loc2 && next1 != paths[a1]->end() && 
			state1->timestep <= state2->timestep && state2->timestep <= next1->timestep)
		{
			return std::make_shared<Conflict>(a1, a2, loc1, -1, state2->timestep);
		}

		auto next2 = state2;
		++next2;
		if (loc1 == loc2 && next2 != paths[a2]->end() &&
			state2->timestep <= state1->timestep && state1->timestep <= next2->timestep)
		{
			return std::make_shared<Conflict>(a1, a2, loc1, -1, state1->timestep);
		}

		int next_loc1 = -1; 
		if (next1 != paths[a1]->end())
			next_loc1 = G.get_location(next1->id);
		if (loc2 == next_loc1 &&
			state1->timestep <= state2->timestep && state2->timestep <= next1->timestep)
		{
			return std::make_shared<Conflict>(a1, a2, loc2, -1, state2->timestep);
		}
		
		int next_loc2 = -1;
		if (next2 != paths[a2]->end())
			next_loc2 = G.get_location(next2->id);
		if (loc1 == next_loc2 && 
			state2->timestep <= state1->timestep && state1->timestep <= next2->timestep)
		{
			return std::make_shared<Conflict>(a1, a2, loc1, -1, state1->timestep);
		}
		
		if (state1->timestep < state2->timestep)
			++state1;
		else if (state1->timestep < state2->timestep)
			++state2;
		else if (next1 == paths[a1]->end())
			++state2;
		else if (next2 == paths[a2]->end())
			++state1;
		else if (next1->timestep < next2->timestep)
			++state1;
		else
			++state2;
	}
	return nullptr;
}
 /*{
	size_t min_path_length = paths[a1]->size() < paths[a2]->size() ? paths[a1]->size() : paths[a2]->size();
	for (size_t timestep = 0; timestep < min_path_length; timestep++)
	{
		int loc1 = getAgentLocation(a1, timestep);
		int loc2 = getAgentLocation(a2, timestep);
		if (k_robust == 0)
        {
            int next_loc1 = getAgentLocation(a1, timestep + 1);
            int next_loc2 = getAgentLocation(a2, timestep + 1);
            if (loc1 == loc2)// vertex conflict
            {
                set.push_back(std::make_shared<Conflict>(a1, a2, loc1, -1, timestep));
            }
            else if (loc1 != next_loc1 && loc1 == next_loc2 && loc2 == next_loc1)// edge conflict
            {
                set.push_back(std::make_shared<Conflict>(a1, a2, loc1, loc2, timestep + 1));
            }
        } else{
		}
	}
	if (paths[a1]->size() != paths[a2]->size()) 
	{// check whether there are conflicts that occur after one agent reaches its goal
		int a1_ = paths[a1]->size() < paths[a2]->size() ? a1 : a2;
		int a2_ = paths[a1]->size() < paths[a2]->size() ? a2 : a1;
		int loc1 = getAgentLocation(a1_, INT_MAX);
		for (size_t timestep = min_path_length; timestep < paths[a2_]->size(); timestep++)
		{
			int loc2 = getAgentLocation(a2_, timestep);
			if (loc1 == loc2)
			{// It's at least a semi conflict			
				set.push_front(std::make_shared<Conflict>(a1_, a2_, loc1, -1, timestep));
			}
		}
	}
}*/

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
	return nullptr;
}

// Generates a boolean reservation table (i.e., conflict avoidance table) for paths (cube of map_size*max_timestep).
// This is used by the low-level ECBS to count possible collisions efficiently
// Note -- we do not include the agent for which we are about to plan for
template<class MyGraph>
void ECBSSearch<MyGraph>::updateReservationTable(size_t max_path_len, int exclude_agent)
{
	cat.clear();
	cat.resize(max_path_len);
	for (int i = 0; i < num_of_agents; i++)
	{
		if (i != exclude_agent && paths[i] != nullptr && !paths[i]->empty())
		{
			for (int step = 1; step < (int)paths[i]->size() - 1; step++)
			{
				if (paths[i]->at(step).id >= 0)
				{
					int loc = G.get_location(paths[i]->at(step).id);
					for (int t = paths[i]->at(step).timestep; t < paths[i]->at(step + 1).timestep; t++)
						cat[t].insert(loc);
					int next_loc = G.get_location(paths[i]->at(step + 1).id);
					if (loc != next_loc)
					{
						for (int t = paths[i]->at(step).timestep + 1; t <= paths[i]->at(step + 1).timestep; t++)
						{
							cat[t].insert(next_loc);
						}
					}
				}
			}
		}
	}
}



//////////////////// GENERATE A NODE ///////////////////////////
// plan a path for an agent in a child node
template<class MyGraph>
void ECBSSearch<MyGraph>::findPathForSingleAgent(ECBSNode*  node, int ag)
{
	// extract all constraints on agent ag
	ECBSNode* curr = node;
	vector < list< int > > cons_vec;
	collectConstraints(curr, ag, cons_vec);
	// build reservation table
	updateReservationTable(node->makespan + 1, ag);
	// find a path w.r.t cons_vec (and prioretize by res_table).
	single_planner.runFocalSearch(G, ag, focal_w, cons_vec, cat);
	LL_num_expanded += single_planner.num_expanded;
	LL_num_generated += single_planner.num_generated;

	node->paths_updated.emplace_back(ag, vector<pathEntry>(single_planner.path),
		single_planner.path_cost, single_planner.min_f_val);
	if (paths[ag]->empty())
	{
		node->g_val = node->g_val - (max_makespan + 1) + single_planner.path_cost;
	}
	else
	{
		node->g_val = node->g_val - paths[ag]->back().timestep + single_planner.path_cost;
	}
	if (!single_planner.path.empty())
	{
		node->makespan = max(node->makespan, single_planner.path.back().timestep);
	}
	node->sum_min_f_vals = node->sum_min_f_vals - ll_min_f_vals[ag] + single_planner.min_f_val;
	paths[ag] = &get<1>(node->paths_updated.back());
	ll_min_f_vals[ag] = single_planner.min_f_val;
}

// plan paths for a child node
template<class MyGraph>
void ECBSSearch<MyGraph>::generateChild(ECBSNode*  node)
{
	findPathForSingleAgent(node, node->agent_id);
	HL_num_generated++;
	node->time_generated = HL_num_generated;
	findConflicts(*node);
	node->num_of_collisions = (int)node->conflicts.size(); //computeNumOfCollidingAgents();

	// update handles
	node->open_handle = open_list.push(node);

	if (node->sum_min_f_vals <= focal_list_threshold)
		node->focal_handle = focal_list.push(node);
	//allNodes_table[n1] = n1;
	allNodes_table.push_back(node);
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

    generateRoot();

	// start is already in the open_list
	while (!focal_list.empty())
	{
		runtime = (double)(std::clock() - start) / CLOCKS_PER_SEC;
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
			solution_makespan = curr->makespan;
			break;
		}

		// generate the two successors that resolve one of the conflicts
		HL_num_expanded++;
		curr->time_expanded = HL_num_expanded;
		int agent1_id, agent2_id, location1, location2, timestep;
		tie(agent1_id, agent2_id, location1, location2, timestep) = *chooseConflict(curr->conflicts);

		auto n1 = new ECBSNode();
		auto n2 = new ECBSNode();
		n1->parent = curr;
		n2->parent = curr;
		n1->g_val = curr->g_val;
		n2->g_val = curr->g_val;
		n1->makespan = curr->makespan;
		n2->makespan = curr->makespan;
		n1->sum_min_f_vals = curr->sum_min_f_vals;
		n2->sum_min_f_vals = curr->sum_min_f_vals;

		n1->agent_id = agent1_id;
		n2->agent_id = agent2_id;
		n1->constraint = make_tuple(location1, location2, timestep, timestep + 1, false);
		n2->constraint = make_tuple(location1, location2, timestep, timestep + 1, false);

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


	runtime = (double)(std::clock() - start) / CLOCKS_PER_SEC; //  get time
	return solution_found;
}

template<class MyGraph>
void ECBSSearch<MyGraph>::generateRoot()
{
	// generate dummy start and update data structures
	dummy_start = new ECBSNode();
	dummy_start->agent_id = -1;
	dummy_start->g_val = 0;
	dummy_start->sum_min_f_vals = 0;
	dummy_start->makespan = 0;
    // initialize paths_found_initially
    paths.resize(num_of_agents, nullptr);
    for (int i = 0; i < num_of_agents; i++)
    {
		vector < list< int > > cons_vec;
		collectConstraints(dummy_start, i, cons_vec);
        updateReservationTable(dummy_start->makespan + 1, i);
		single_planner.runFocalSearch(G, i, focal_w, cons_vec, cat);
        paths[i] = new vector<pathEntry>(single_planner.path);
		ll_min_f_vals[i] = single_planner.min_f_val;
		paths_costs[i] = single_planner.path_cost;
        LL_num_expanded += single_planner.num_expanded;
        LL_num_generated += single_planner.num_generated;
		dummy_start->g_val += paths_costs[i];
		dummy_start->sum_min_f_vals += ll_min_f_vals[i];
		if (!single_planner.path.empty())
			dummy_start->makespan = max(dummy_start->makespan, single_planner.path.back().timestep);
    }

	paths_found_initially = paths;
   ll_min_f_vals_found_initially = ll_min_f_vals;
    paths_costs_found_initially = paths_costs;

    findConflicts(*dummy_start);
    dummy_start->num_of_collisions = (int)dummy_start->conflicts.size();
    dummy_start->open_handle = open_list.push(dummy_start);
    dummy_start->focal_handle = focal_list.push(dummy_start);
    HL_num_generated++;
    dummy_start->time_generated = HL_num_generated;

    allNodes_table.push_back(dummy_start);

    min_sum_f_vals = dummy_start->sum_min_f_vals;
    focal_list_threshold = focal_w * dummy_start->sum_min_f_vals;
}

template<class MyGraph>
ECBSSearch<MyGraph>::ECBSSearch(const MyGraph& G, double focal_w, int makespan, bool disjointSplitting, double cutoffTime) :
	focal_w(focal_w), max_makespan(makespan), disjointSplitting(disjointSplitting), G(G),
	single_planner(G.map_size(), makespan)
{
	time_limit = cutoffTime;
	num_of_agents = (int)G.start_ids.size();
	map_size = G.map_size();
	ll_min_f_vals = vector <int>(num_of_agents);
	paths_costs = vector <int>(num_of_agents);
	ll_min_f_vals_found_initially = vector <int>(num_of_agents);
	paths_costs_found_initially = vector <int>(num_of_agents);
}

template<class MyGraph>
ECBSSearch<MyGraph>::~ECBSSearch()
{
	for (size_t i = 0; i < paths_found_initially.size(); i++)
		delete (paths_found_initially[i]);
	for (const auto& n : allNodes_table)
		delete n;
}
