#include "CorridorReasoning.h"


bool validMove(int curr, int next, int map_cols, int map_size)
{
	if (next < 0 || next >= map_size)
		return false;
	return getMahattanDistance(curr, next, map_cols) < 2;
}

int getMahattanDistance(int loc1, int loc2, int map_cols)
{
	int loc1_x = loc1 / map_cols;
	int loc1_y = loc1 % map_cols;
	int loc2_x = loc2 / map_cols;
	int loc2_y = loc2 % map_cols;
	return std::abs(loc1_x - loc2_x) + std::abs(loc1_y - loc2_y);
}


int getDegree(int loc, const bool*map, int num_col, int map_size)
{
	if (loc < 0 || loc >= map_size || map[loc])
		return -1;
	int degree = 0;
	if (0 < loc - num_col && !map[loc - num_col])
		degree++;
	if (loc + num_col < map_size && !map[loc + num_col])
		degree++;
	if (loc % num_col > 0 && !map[loc - 1])
		degree++;
	if (loc % num_col < num_col - 1 && !map[loc + 1])
		degree++;
	return degree;
}


int getCorridorLength(const std::vector<PathEntry>& path, int t_start, int loc_end)
{
	int prev = path[t_start].location;
	int length = 0; // distance to the start location
	int t = t_start;
	while (prev != loc_end)
	{
		t++;
		int curr = path[t].location;
		if (prev != curr) // move forward
        {
            length++;
            prev = curr;
        }
	}
    return length + 2; // +2 because the start and end locations are one cell inside the corridor
}

template<class Map>
int CorridorReasoning<Map>::getEnteringTime(const std::vector<PathEntry>& path, const std::vector<PathEntry>& path2, int t,
	Map* map)
{
    assert(t < path.size());
	int loc = path[t].location;
	while (loc != path.front().location && loc != path2.front().location &&
	    loc != path.back().location && loc != path2.back().location &&
		map->getDegree(loc) == 2)
	{
		t--;
		loc = path[t].location;
	}
	return t + 1;
}


template<class Map>
int CorridorReasoning<Map>::getExitTime(const std::vector<PathEntry>& path, const std::vector<PathEntry>& path2, int t,
	Map* map)
{
    assert(t < path.size());
	int loc = path[t].location;
	while (loc != path.front().location && loc != path2.front().location &&
	    loc != path.back().location && loc != path2.back().location &&
		map->getDegree(loc) == 2)
	{
		t++;
		loc = path[t].location;
	}
	return t - 1;
}

//with heading info. not using
template<class Map>
int CorridorReasoning<Map>::getBypassLength(int start, int end, std::pair<int, int> blocked,  Map* my_map, int num_col, int map_size,int start_heading)
{
	int length = INT_MAX;
	// generate a heap that can save nodes (and a open_handle)
	boost::heap::fibonacci_heap< LLNode*, boost::heap::compare<LLNode::compare_node> > heap;
	boost::heap::fibonacci_heap< LLNode*, boost::heap::compare<LLNode::compare_node> >::handle_type open_handle;
	// generate hash_map (key is a node pointer, data is a node handler,
	//                    NodeHasher is the hash function to be used,
	//                    eqnode is used to break ties when hash values are equal)
    typedef boost::unordered_set<LLNode*, LLNode::NodeHasher, LLNode::eqnode> hashtable_t;
    hashtable_t nodes;
    hashtable_t::iterator it; // will be used for find()

	LLNode* root = new LLNode(start, 0, getMahattanDistance(start, end, num_col), nullptr, 0);
	root->heading = start_heading;
	root->open_handle = heap.push(root);  // add root to heap
	nodes.insert(root);       // add root to hash_table (nodes)
	int moves_offset[4] = { 1, -1, num_col, -num_col };
	LLNode* curr = nullptr;
	int time_generated = 0;
	while (!heap.empty())
	{
		curr = heap.top();
		heap.pop();
		if (curr->loc == end)
		{
			length = curr->g_val;
			break;
		}
		list<Transition> transitions;
		my_map->get_transitions(transitions, curr->loc, curr->heading, false);

		for (const auto& move : transitions)
		{
			int next_loc = move.location;
			time_generated += 1;

			if ((curr->loc == blocked.first && next_loc == blocked.second) ||
				(curr->loc == blocked.second && next_loc == blocked.first)) // use the prohibited edge
			{
				continue;
			}
			int next_g_val = curr->g_val + 1;
			LLNode* next = new LLNode(next_loc, next_g_val, getMahattanDistance(next_loc, end, num_col), nullptr, 0);
			int next_heading = move.heading;
			next->heading = next_heading;
			next->actionToHere = move.heading;
			next->time_generated = time_generated;

			it = nodes.find(next);
			if (it == nodes.end())
			{  // add the newly generated node to heap and hash table
				next->open_handle = heap.push(next);
				nodes.insert(next);
			}
			else {  // update existing node's g_val if needed (only in the heap)
				delete(next);  // not needed anymore -- we already generated it before
				LLNode* existing_next = (*it);
				if (existing_next->g_val > next_g_val)
				{
					existing_next->g_val = next_g_val;
					heap.update(open_handle);
				}
			}
			
		}
	}
	for (it = nodes.begin(); it != nodes.end(); it++)
	{
		delete (*it);
	}
	return length;
}

//with heuristics table. not using
template<class Map>
int CorridorReasoning<Map>::getBypassLength(int start, int end, std::pair<int, int> blocked,  Map* my_map, int num_col, int map_size, ConstraintTable& constraint_table, int upper_bound, std::vector<hvals> restable, int start_heading)
{
	int length = INT_MAX;
	// generate a heap that can save nodes (and a open_handle)
	boost::heap::fibonacci_heap< LLNode*, boost::heap::compare<LLNode::compare_node> > heap;
	boost::heap::fibonacci_heap< LLNode*, boost::heap::compare<LLNode::compare_node> >::handle_type open_handle;
	// generate hash_map (key is a node pointer, data is a node handler,
	//                    NodeHasher is the hash function to be used,
	//                    eqnode is used to break ties when hash values are equal)
    typedef boost::unordered_set<LLNode*, LLNode::NodeHasher, LLNode::eqnode> hashtable_t;
    hashtable_t nodes;
    hashtable_t::iterator it; // will be used for find()

	LLNode* root = new LLNode(start, 0, getMahattanDistance(start, end, num_col), nullptr, 0);
	root->heading = start_heading;
	root->open_handle = heap.push(root);  // add root to heap
	nodes.insert(root);       // add root to hash_table (nodes)
	int moves_offset[5] = { 1, -1, num_col, -num_col, 0};
	LLNode* curr = nullptr;
	int time_generated = 0;
	while (!heap.empty())
	{
		curr = heap.top();
		heap.pop();
		if (curr->loc == end)
		{
			length = curr->g_val;
			break;
		}
		list<Transition> transitions;
		my_map->get_transitions(transitions, curr->loc, curr->heading, false);

		for (const auto& move : transitions)
		{
			int next_loc = move.location;
			time_generated += 1;

			int next_timestep = curr->timestep + 1;
			if ( !constraint_table.is_constrained(next_loc, next_timestep) &&
				!constraint_table.is_constrained(curr->loc * map_size + next_loc, next_timestep))
			{  // if that grid is not blocked
				if ((curr->loc == blocked.first && next_loc == blocked.second) ||
					(curr->loc == blocked.second && next_loc == blocked.first)) // use the prohibited edge
				{
					continue;
				}
				int next_heading = move.heading;

				int next_g_val = curr->g_val + 1;
				int next_h_val = restable[next_loc].get_hval(next_heading);
				if (next_g_val + next_h_val >= upper_bound) // the cost of the path is larger than the upper bound
					continue;
				LLNode* next = new LLNode(next_loc, next_g_val, next_h_val, nullptr, next_timestep);

				next->heading = next_heading;
				next->actionToHere = move.heading;
				next->time_generated = time_generated;

				it = nodes.find(next);
				if (it == nodes.end())
				{  // add the newly generated node to heap and hash table
					next->open_handle = heap.push(next);
					nodes.insert(next);
				}
				else {  // update existing node's g_val if needed (only in the heap)
					delete(next);  // not needed anymore -- we already generated it before
					LLNode* existing_next = (*it);
					if (existing_next->g_val > next_g_val)
					{
						existing_next->g_val = next_g_val;
						existing_next->timestep = next_timestep;
						heap.update(open_handle);
					}
				}
			}
		}
	}
	for (it = nodes.begin(); it != nodes.end(); it++)
	{
		delete (*it);
	}
	return length;
}

//for corridor2, using for flatland
template<class Map>
int CorridorReasoning<Map>::getBypassLength(int start, int end,int start_heading,int end_heading, std::pair<int, int> blocked, Map* my_map, int num_col, int map_size,
        ConstraintTable& constraint_table, const std::vector<hvals>& goalHeuTable, int upper_bound, PathEntry& start_entry,float speed)
{
    if(upper_bound < 0)
        upper_bound = -upper_bound;
	int length = INT_MAX;
	// generate a heap that can save nodes (and a open_handle)
	boost::heap::fibonacci_heap< LLNode*, boost::heap::compare<LLNode::compare_node> > heap;
	boost::heap::fibonacci_heap< LLNode*, boost::heap::compare<LLNode::compare_node> >::handle_type open_handle;
	// generate hash_map (key is a node pointer, data is a node handler,
	//                    NodeHasher is the hash function to be used,
	//                    eqnode is used to break ties when hash values are equal)
    typedef boost::unordered_set<LLNode*, LLNode::NodeHasher, LLNode::eqnode> hashtable_t;
    hashtable_t nodes;
    hashtable_t::iterator it; // will be used for find()

	LLNode* root = new LLNode(-1, 0, getMahattanDistance(start, end, num_col), nullptr, 0);
	root->heading = start_heading;
	root->position_fraction = 0;

	root->exit_heading = start_entry.exit_heading;
	root->exit_loc = start_entry.exit_loc;

	root->open_handle = heap.push(root);  // add root to heap

	int start_h_val = abs(goalHeuTable[end].get_hval(end_heading) - goalHeuTable[start].get_hval(start_heading)) / speed;
	if (root->exit_loc >= 0 && speed < 1 ) {
		int h1 = abs(goalHeuTable[end].get_hval(end_heading) - goalHeuTable[start].get_hval(start_heading)) ;
		int h2 = abs(goalHeuTable[end].get_hval(end_heading) - goalHeuTable[root->exit_loc].get_hval(root->exit_heading)) ;
		start_h_val = h1 / speed
			- (h2 - h1)*(root->position_fraction/speed);

	}
	root->h_val = start_h_val;
	

	nodes.insert(root);       // add root to hash_table (nodes)
	int moves_offset[5] = { 1, -1, num_col, -num_col, 0 };
	LLNode* curr = nullptr;
	int time_generated = 0;
	int num_nodes=0;
	while (!heap.empty())
	{
        num_nodes++;
		curr = heap.top();
		heap.pop();

		if (curr->loc == end && goalHeuTable[end].get_hval(end_heading) >= goalHeuTable[curr->loc].get_hval(curr->heading))
		{
			length = curr->g_val;
			break;
		}
		list<Transition> transitions;
		if(curr->loc == -1){
            Transition move;
            move.location = -1;
            move.heading = curr->heading;
            move.position_fraction = curr->position_fraction;
            move.exit_loc = curr->exit_loc;
            move.exit_heading = curr->exit_heading;
            transitions.push_back(move);

            Transition move2;
            move2.location = start;
            move2.heading = curr->heading;
            move2.position_fraction = curr->position_fraction;
            move2.exit_loc = curr->exit_loc;
            move2.exit_heading = curr->exit_heading;
            transitions.push_back(move2);

        }
		else if (curr->position_fraction +speed >= 0.97) {
			if (curr->position_fraction == 0)
				 my_map->get_transitions(transitions, curr->loc, curr->heading, false);
			else {
				Transition move;
				move.location = curr->exit_loc;
				move.heading = curr->exit_heading;
				move.position_fraction = 0;
				transitions.push_back(move);
			}
		}
		else if (curr->position_fraction == 0) {
			 my_map->get_exits(transitions, curr->loc, curr->heading, speed, false);

		}
		else { //<0.97 and po_frac not 0


			Transition move2;
			move2.location = curr->loc;
			move2.heading = curr->heading;
			move2.position_fraction = curr->position_fraction + speed;
			move2.exit_loc = curr->exit_loc;
			move2.exit_heading = curr->exit_heading;
			transitions.push_back(move2);
		}




		for (const auto& move : transitions)
		{
			int next_loc = move.location;
			time_generated += 1;

			float next_position_fraction = move.position_fraction;

			int next_timestep = curr->timestep + 1;

			if (!constraint_table.is_constrained(next_loc, next_timestep))  // &&
				// !constraint_table.is_constrained(curr->loc * map_size + next_loc, next_timestep))
			{  // if that grid is not blocked

                if ((curr->loc == blocked.first && next_loc == blocked.second) ||
					(curr->loc == blocked.second && next_loc == blocked.first)) // use the prohibited edge
				{
					continue;
				}

                int next_heading = move.heading;

				int next_g_val = curr->g_val + 1;
				int next_h_val;
				if (next_loc == -1)
				    next_h_val = curr->h_val;
                else
                    next_h_val = abs(goalHeuTable[end].get_hval(end_heading) - goalHeuTable[next_loc].get_hval(next_heading))/speed;
				if (next_loc != -1 && move.exit_loc >= 0 && speed < 1) {
					int h1 = abs(goalHeuTable[end].get_hval(end_heading) - goalHeuTable[next_loc].get_hval(next_heading));
					int h2 = abs(goalHeuTable[end].get_hval(end_heading) - goalHeuTable[move.exit_loc].get_hval(move.exit_heading));
					next_h_val = h1 / speed
						- (h2 - h1)*(move.position_fraction/speed);

				}

                if ((next_g_val + next_h_val*speed) >= upper_bound) // the cost of the path is larger than the upper bound
					continue;

                LLNode* next = new LLNode(next_loc, next_g_val, next_h_val, nullptr, next_timestep);

				next->heading = next_heading;
				next->actionToHere = move.heading;
				next->time_generated = time_generated;
				next->position_fraction = next_position_fraction;
				next->exit_heading = move.exit_heading;
				next->exit_loc = move.exit_loc;
				/*std::cout << "current: (" << curr->loc << "," << curr->heading << "," << curr->getFVal() <<","<<curr->g_val<<","<<curr->h_val<<","<<curr->position_fraction << ") "
					<< "next: (" << next->loc << "," << next->heading << "," << next->getFVal() << "," << next->g_val << "," << next->h_val <<","<<next->position_fraction<< ")"
					<< " goal: "<< end<< std::endl;*/

				it = nodes.find(next);
				if (it == nodes.end())
				{  // add the newly generated node to heap and hash table
					next->open_handle = heap.push(next);
					nodes.insert(next);
				}
				else {  // update existing node's g_val if needed (only in the heap)
					delete(next);  // not needed anymore -- we already generated it before
					LLNode* existing_next = (*it);
					if (existing_next->g_val > next_g_val)
					{
						existing_next->g_val = next_g_val;
						existing_next->timestep = next_timestep;
						heap.update(open_handle);
					}
				}
			}
		}
	}
	for (it = nodes.begin(); it != nodes.end(); it++)
	{
		delete (*it);
	}
    return length;
}



bool isConstrained(int curr_id, int next_id, int next_timestep, const std::vector< std::list< std::pair<int, int> > >* cons)
{
	if (cons == nullptr)
		return false;
	// check vertex constraints (being in next_id at next_timestep is disallowed)
	if (next_timestep < static_cast<int>(cons->size()))
	{
		for (std::list< std::pair<int, int> >::const_iterator it = cons->at(next_timestep).begin(); it != cons->at(next_timestep).end(); ++it)
		{
			if ((std::get<0>(*it) == next_id && std::get<1>(*it) < 0)//vertex constraint
				|| (std::get<0>(*it) == curr_id && std::get<1>(*it) == next_id)) // edge constraint
				return true;
		}
	}
	return false;
};
template class CorridorReasoning<MapLoader>;
template class CorridorReasoning<FlatlandLoader>;

//
//template  int CorridorReasoning::getEnteringTime<MapLoader>(const std::vector<PathEntry>&, const std::vector<PathEntry>&, int, const MapLoader*);
//template  int CorridorReasoning::getEnteringTime<FlatlandLoader>(const std::vector<PathEntry>&, const std::vector<PathEntry>& , int , const FlatlandLoader* );
//template  int CorridorReasoning::getBypassLength<MapLoader>(int , int , std::pair<int, int> , const MapLoader* , int , int , int );
//template  int CorridorReasoning::getBypassLength<MapLoader>(int , int , std::pair<int, int> , const MapLoader* , int , int , ConstraintTable& , int , int);
//template  int CorridorReasoning::getBypassLength<FlatlandLoader>(int , int , std::pair<int, int> , const FlatlandLoader* , int , int , int );
//template  int CorridorReasoning::getBypassLength<FlatlandLoader>(int , int , std::pair<int, int> , const FlatlandLoader* , int , int , ConstraintTable& , int , int );
