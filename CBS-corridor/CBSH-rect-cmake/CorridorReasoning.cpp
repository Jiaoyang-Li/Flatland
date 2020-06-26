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


int getCorridorLength(const std::vector<PathEntry>& path, int t_start, int loc_end, std::pair<int, int>& edge)
{
	int curr = path[t_start].location;
	int next;
	int prev = -1;
	int length = 0; // distance to the start location
	int t = t_start;
	bool moveForward = true;
	bool updateEdge = false;
	while (curr != loc_end)
	{
		t++;
		next = path[t].location;
		if (next == curr) // wait
			continue;
		else if (next == prev) // turn aournd
			moveForward = !moveForward;
		if (moveForward)
		{
			if (!updateEdge)
			{
				edge = std::make_pair(curr, next);
				updateEdge = true;
			}
			length++;
		}
		else
			length--;
		prev = curr;
		curr = next;
	}
	return length;
}

template<class Map>
int CorridorReasoning<Map>::getEnteringTime(const std::vector<PathEntry>& path, const std::vector<PathEntry>& path2, int t,
	Map* map)
{
	if (t >= path.size())
		t = path.size() - 1;
	int loc = path[t].location;
	while (loc != path.front().location && loc != path2.back().location &&
		map->getDegree(loc) == 2)
	{
		t--;
		loc = path[t].location;
	}
	return t;
}

template<class Map>
int CorridorReasoning<Map>::getExitTime(const std::vector<PathEntry>& path, const std::vector<PathEntry>& path2, int t,
	Map* map)
{
	if (t >= path.size())
		t = path.size() - 1;
	int loc = path[t].location;
	while (loc != path.front().location && loc != path2.back().location &&
		map->getDegree(loc) == 2)
	{
		t++;
		loc = path[t].location;
	}
	return t;
}


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
	google::dense_hash_map<LLNode*, fibonacci_heap<LLNode*, boost::heap::compare<LLNode::compare_node> >::handle_type, LLNode::NodeHasher, LLNode::eqnode> nodes;
	nodes.set_empty_key(NULL);
	google::dense_hash_map<LLNode*, fibonacci_heap<LLNode*, boost::heap::compare<LLNode::compare_node> >::handle_type, LLNode::NodeHasher, LLNode::eqnode>::iterator it; // will be used for find()

	LLNode* root = new LLNode(start, 0, getMahattanDistance(start, end, num_col), NULL, 0);
	root->heading = start_heading;
	root->open_handle = heap.push(root);  // add root to heap
	nodes[root] = root->open_handle;       // add root to hash_table (nodes)
	int moves_offset[4] = { 1, -1, num_col, -num_col };
	LLNode* curr = NULL;
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
		vector<Transition> transitions = my_map->get_transitions(curr->loc, curr->heading, false);

		for (const auto move : transitions)
		{
			int next_loc = move.first;
			time_generated += 1;

			if ((curr->loc == blocked.first && next_loc == blocked.second) ||
				(curr->loc == blocked.second && next_loc == blocked.first)) // use the prohibited edge
			{
				continue;
			}
			int next_g_val = curr->g_val + 1;
			LLNode* next = new LLNode(next_loc, next_g_val, getMahattanDistance(next_loc, end, num_col), NULL, 0);
			int next_heading;

			if (curr->heading == -1) //heading == 4 means no heading info
				next_heading = -1;
			else
				if (move.second == 4) //move == 4 means wait
					next_heading = curr->heading;
				else
					next_heading = move.second;
			next->heading = next_heading;
			next->actionToHere = move.second;
			next->time_generated = time_generated;

			it = nodes.find(next);
			if (it == nodes.end())
			{  // add the newly generated node to heap and hash table
				next->open_handle = heap.push(next);
				nodes[next] = next->open_handle;
			}
			else {  // update existing node's g_val if needed (only in the heap)
				delete(next);  // not needed anymore -- we already generated it before
				LLNode* existing_next = (*it).first;
				open_handle = (*it).second;
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
		delete (*it).first;
	}
	return length;
}

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
	google::dense_hash_map<LLNode*, fibonacci_heap<LLNode*, boost::heap::compare<LLNode::compare_node> >::handle_type, LLNode::NodeHasher, LLNode::eqnode> nodes;
	nodes.set_empty_key(NULL);
	google::dense_hash_map<LLNode*, fibonacci_heap<LLNode*, boost::heap::compare<LLNode::compare_node> >::handle_type, LLNode::NodeHasher, LLNode::eqnode>::iterator it; // will be used for find()

	LLNode* root = new LLNode(start, 0, getMahattanDistance(start, end, num_col), NULL, 0);
	root->heading = start_heading;
	root->open_handle = heap.push(root);  // add root to heap
	nodes[root] = root->open_handle;       // add root to hash_table (nodes)
	int moves_offset[5] = { 1, -1, num_col, -num_col, 0};
	LLNode* curr = NULL;
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
		vector<Transition> transitions = my_map->get_transitions(curr->loc, curr->heading, false);

		for (const auto move : transitions)
		{
			int next_loc = move.first;
			time_generated += 1;

			int next_timestep = curr->timestep + 1;
			if (constraint_table.latest_timestep <= curr->timestep)
			{
				if (move.second == 4)
				{
					continue;
				}
				next_timestep--;
			}
			
			if ( !constraint_table.is_constrained(next_loc, next_timestep) &&
				!constraint_table.is_constrained(curr->loc * map_size + next_loc, next_timestep))
			{  // if that grid is not blocked
				if ((curr->loc == blocked.first && next_loc == blocked.second) ||
					(curr->loc == blocked.second && next_loc == blocked.first)) // use the prohibited edge
				{
					continue;
				}
				int next_heading;

				if (curr->heading == -1) //heading == 4 means no heading info
					next_heading = -1;
				else
					if (move.second == 4) //move == 4 means wait
						next_heading = curr->heading;
					else
						next_heading = move.second;

				int next_g_val = curr->g_val + 1;
				int next_h_val = restable[next_loc].get_hval(next_heading);
				if (next_g_val + next_h_val >= upper_bound) // the cost of the path is larger than the upper bound
					continue;
				LLNode* next = new LLNode(next_loc, next_g_val, next_h_val, NULL, next_timestep);

				next->heading = next_heading;
				next->actionToHere = move.second;
				next->time_generated = time_generated;

				it = nodes.find(next);
				if (it == nodes.end())
				{  // add the newly generated node to heap and hash table
					next->open_handle = heap.push(next);
					nodes[next] = next->open_handle;
				}
				else {  // update existing node's g_val if needed (only in the heap)
					delete(next);  // not needed anymore -- we already generated it before
					LLNode* existing_next = (*it).first;
					open_handle = (*it).second;
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
		delete (*it).first;
	}
	return length;
}

//for corridor2
template<class Map>
int CorridorReasoning<Map>::getBypassLength(int end, std::pair<int, int> blocked, Map* my_map, int num_col, int map_size, ConstraintTable& constraint_table, int upper_bound, PathEntry& start_entry,float speed,int max_malfunction)
{
	int length = INT_MAX;
	// generate a heap that can save nodes (and a open_handle)
	boost::heap::fibonacci_heap< LLNode*, boost::heap::compare<LLNode::compare_node> > heap;
	boost::heap::fibonacci_heap< LLNode*, boost::heap::compare<LLNode::compare_node> >::handle_type open_handle;
	// generate hash_map (key is a node pointer, data is a node handler,
	//                    NodeHasher is the hash function to be used,
	//                    eqnode is used to break ties when hash values are equal)
	google::dense_hash_map<LLNode*, fibonacci_heap<LLNode*, boost::heap::compare<LLNode::compare_node> >::handle_type, LLNode::NodeHasher, LLNode::eqnode> nodes;
	nodes.set_empty_key(NULL);
	google::dense_hash_map<LLNode*, fibonacci_heap<LLNode*, boost::heap::compare<LLNode::compare_node> >::handle_type, LLNode::NodeHasher, LLNode::eqnode>::iterator it; // will be used for find()

	LLNode* root = new LLNode(start_entry.location, 0, getMahattanDistance(start_entry.location, end, num_col), NULL, 0);
	root->heading = start_entry.actionToHere;
	root->malfunction_left = start_entry.malfunction_left;
	root->next_malfunction = start_entry.next_malfunction;
	root->position_fraction = start_entry.position_fraction;

	root->exit_heading = start_entry.exit_heading;
	root->exit_loc = start_entry.exit_loc;

	root->open_handle = heap.push(root);  // add root to heap

	int start_h_val = getMahattanDistance(root->loc, end, num_col) / speed;
	if (root->exit_loc >= 0 && speed < 1) {
		int h1 = getMahattanDistance(root->loc, end, num_col);
		int h2 = getMahattanDistance(root->exit_loc, end, num_col);;
		start_h_val = h1 / speed
			- (h2 - h1)*speed;

	}
	root->h_val = start_h_val;
	

	nodes[root] = root->open_handle;       // add root to hash_table (nodes)
	int moves_offset[5] = { 1, -1, num_col, -num_col, 0 };
	LLNode* curr = NULL;
	int time_generated = 0;
	while (!heap.empty())
	{
		curr = heap.top();
		heap.pop();
		if (curr->loc == end && curr->position_fraction+speed>0.97)
		{
			length = curr->g_val;
			break;
		}
		vector<Transition> transitions;
		if (curr->malfunction_left > 0) {
			Transition move;
			move.first = curr->loc;
			move.second = 4;
			move.position_fraction = curr->position_fraction;
			move.exit_loc = curr->exit_loc;
			move.exit_heading = curr->exit_heading;
			transitions.push_back(move);
		}
		else if (curr->position_fraction +speed >= 0.97) {
			if (curr->position_fraction == 0)
				transitions = my_map->get_transitions(curr->loc, curr->heading, false);
			else {
				Transition move;
				move.first = curr->exit_loc;
				move.second = curr->exit_heading;
				move.position_fraction = 0;
				transitions.push_back(move);
			}
		}
		else if (curr->position_fraction == 0) {
			transitions = my_map->get_exits(curr->loc, curr->heading, speed, false);

		}
		else { //<0.97 and po_frac not 0


			Transition move2;
			move2.first = curr->loc;
			move2.second = 4;
			move2.position_fraction = curr->position_fraction + speed;
			move2.exit_loc = curr->exit_loc;
			move2.exit_heading = curr->exit_heading;
			transitions.push_back(move2);
		}

		//bool allConstrained=true;
		//cout << "****" << endl;
		//cout << "current: (" << curr->loc << "," << curr->heading << "," << curr->getFVal() << "," << curr->g_val << "," << curr->h_val << "," << curr->position_fraction << ") " << endl;
		//if (transitions.size() == 0) {
		//	cout << "no children" << endl;
		//}
		//for (const auto move : transitions)
		//{
		//	int next_timestep = curr->timestep + 1;
		//	cout << move.first << "," << move.second << "," << move.position_fraction << "," << move.exit_heading << "," << move.exit_loc << endl;
		//	if (!constraint_table.is_constrained(move.first, next_timestep) &&
		//		!constraint_table.is_constrained(curr->loc * map_size + move.first, next_timestep))
		//	{
		//		allConstrained = false;
		//	}
		//	else {
		//		cout << "node constrained" << endl;
		//	}
		//}
		//if (allConstrained) {
		//	cout << "all constrainted" << endl;

		//}


		for (const auto move : transitions)
		{
			int next_loc = move.first;
			time_generated += 1;

			int next_malfunction_left = curr->malfunction_left > 0 ? curr->malfunction_left - 1 : curr->malfunction_left;
			int next_next_malfuntion = (curr->next_malfunction > 0 && curr->malfunction_left == 0) ? curr->next_malfunction - 1 : curr->next_malfunction;
			if (curr->parent != NULL && next_next_malfuntion == 0 && curr->next_malfunction == 1) {
				next_malfunction_left = max_malfunction;
			}
			float next_position_fraction = move.position_fraction;

			int next_timestep = curr->timestep + 1;
			//if (constraint_table.latest_timestep <= curr->timestep)
			//{
			//	if (move.second == 4)
			//	{
			//		continue;
			//	}
			//	next_timestep--;
			//}

			if (!constraint_table.is_constrained(next_loc, next_timestep) &&
				!constraint_table.is_constrained(curr->loc * map_size + next_loc, next_timestep))
			{  // if that grid is not blocked
				if ((curr->loc == blocked.first && next_loc == blocked.second) ||
					(curr->loc == blocked.second && next_loc == blocked.first)) // use the prohibited edge
				{
					continue;
				}
				int next_heading;

				if (curr->heading == -1) //heading == 4 means no heading info
					next_heading = -1;
				else
					if (move.second == 4) //move == 4 means wait
						next_heading = curr->heading;
					else
						next_heading = move.second;

				int next_g_val = curr->g_val + 1;
				int next_h_val = getMahattanDistance(next_loc, end, num_col)/speed;
				if (move.exit_loc >= 0 && speed < 1) {
					int h1 = getMahattanDistance(next_loc, end, num_col);
					int h2 = getMahattanDistance(move.exit_loc, end, num_col);
					next_h_val = h1 / speed
						- (h2 - h1)*speed;

				}
				/*if (move.exit_loc >= 0) {
					int next_h_val = getMahattanDistance(move.exit_loc, end, num_col); 

				}*/
				if (next_g_val + next_h_val >= upper_bound) // the cost of the path is larger than the upper bound
					continue;
				LLNode* next = new LLNode(next_loc, next_g_val, next_h_val, NULL, next_timestep);

				next->heading = next_heading;
				next->actionToHere = move.second;
				next->time_generated = time_generated;
				next->next_malfunction = next_next_malfuntion;
				next->malfunction_left = next_malfunction_left;
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
					nodes[next] = next->open_handle;
				}
				else {  // update existing node's g_val if needed (only in the heap)
					delete(next);  // not needed anymore -- we already generated it before
					LLNode* existing_next = (*it).first;
					open_handle = (*it).second;
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
		delete (*it).first;
	}
	return length;
}



bool isConstrained(int curr_id, int next_id, int next_timestep, const std::vector< std::list< std::pair<int, int> > >* cons)
{
	if (cons == NULL)
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
