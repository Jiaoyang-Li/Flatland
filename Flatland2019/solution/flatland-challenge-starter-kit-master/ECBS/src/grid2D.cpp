#include "grid2D.h"

bool Grid2D::load_map(string fname)
{
  string line;
  ifstream myfile (fname.c_str());
  if (myfile.is_open()) 
  {
    getline (myfile,line);
    boost::char_separator<char> sep(",");
	boost::tokenizer< boost::char_separator<char> > tok(line, sep);
	boost::tokenizer< boost::char_separator<char> >::iterator beg=tok.begin();
    rows = atoi ( (*beg).c_str() ); // read number of rows
    beg++;
    cols = atoi ( (*beg).c_str() ); // read number of cols
    my_map.resize(rows * cols);
    // read map
    for (int i=0; i<rows; i++) {
		getline (myfile, line);
		for (int j=0; j<cols; j++) {
		  my_map[cols*i + j] = (line[j] != '.');
		}
    }
    myfile.close();
    // initialize moves_offset array
    moves_offset[Grid2D::valid_moves_t::WAIT_MOVE] = 0;
    moves_offset[Grid2D::valid_moves_t::NORTH] = -cols;
    moves_offset[Grid2D::valid_moves_t::EAST] = 1;
    moves_offset[Grid2D::valid_moves_t::SOUTH] = cols;
    moves_offset[Grid2D::valid_moves_t::WEST] = -1;
	return true;
  }
  return false;
}

bool Grid2D::load_agents(string fname)
{
	string line;

	ifstream myfile(fname.c_str());

	if (myfile.is_open())
	{
		getline(myfile, line);
		boost::char_separator<char> sep(",");
		boost::tokenizer< boost::char_separator<char> > tok(line, sep);
		boost::tokenizer< boost::char_separator<char> >::iterator beg = tok.begin();
		int num_of_agents = atoi((*beg).c_str());
		start_ids.resize(num_of_agents);
		goal_ids.resize(num_of_agents);
		for (int i = 0; i<num_of_agents; i++)
		{
			getline(myfile, line);
			boost::tokenizer< boost::char_separator<char> > col_tok(line, sep);
			boost::tokenizer< boost::char_separator<char> >::iterator c_beg = col_tok.begin();
			// read start [row,col] for agent i
			int row = atoi((*c_beg).c_str());
			c_beg++;
			int col = atoi((*c_beg).c_str());
			start_ids[i] = linearize_coordinate(row, col);
			// read goal [row,col] for agent i
			c_beg++;
			row = atoi((*c_beg).c_str());
			c_beg++;
			col = atoi((*c_beg).c_str());
			goal_ids[i] = linearize_coordinate(row, col);
		}
		myfile.close();
		return true;
	}
	return false;
}

list<int> Grid2D::children_vertices(int vertex_id) const
{
	list<int> vertices;
	for (int direction = 0; direction < 5; direction++)
	{
		int next_id = vertex_id + moves_offset[direction];
		if (0 <= next_id && next_id < cols * rows && !my_map[next_id] && abs(next_id % cols - vertex_id % cols) < 2)
			vertices.push_back(next_id);
	}
	return vertices;
}


void Grid2D::preprocessing_heuristics()
{
	size_t num_of_agents = start_ids.size();
	heuristics.resize(num_of_agents);
	for (size_t i = 0; i < num_of_agents; i++)
	{
		compute_heuristics(goal_ids[i], heuristics[i]);
	}
}

// compute low-level heuristics
void Grid2D::compute_heuristics(int goal_location, vector<int>& heuristics)
{
	struct MyNode
	{
		int loc;
		int g_val;
		bool in_openlist;

		// the following is used to comapre nodes in the OPEN list
		struct compare_node
		{
			// returns true if n1 > n2 (note -- this gives us *min*-heap).
			bool operator()(const MyNode* n1, const MyNode* n2) const
			{
				return n1->g_val >= n2->g_val;
			}
		};  // used by OPEN (heap) to compare nodes (top of the heap has min g-val)

			// define a typedefs for handles to the heaps (allow up to quickly update a node in the heap)
		typedef fibonacci_heap< MyNode*, boost::heap::compare<MyNode::compare_node> >
			::handle_type open_handle_t;

		open_handle_t open_handle;

		MyNode() {}
		MyNode(int loc, int g_val, bool in_openlist = false) : loc(loc), g_val(g_val), in_openlist(in_openlist) {}

		// The following is used for checking whether two nodes are equal
		// we say that two nodes, s1 and s2, are equal if
		// both agree on the location and timestep
		struct eqnode
		{
			bool operator()(const MyNode* s1, const MyNode* s2) const
			{
				return (s1 == s2) || (s1 && s2 &&
					s1->loc == s2->loc);
			}
		};

		// The following is used for generating the hash value of a nodes
		// /* TODO:  */his is needed because otherwise we'll have to define the specilized template inside std namespace
		struct NodeHasher
		{
			std::size_t operator()(const MyNode* n) const
			{
				return std::hash<int>()(n->loc);
			}
		};
	};

	int root_location = goal_location;
	// generate a heap that can save nodes (and a open_handle)
	fibonacci_heap< MyNode*, compare<MyNode::compare_node> > heap;
	fibonacci_heap< MyNode*, compare<MyNode::compare_node> >::handle_type open_handle;
	// generate hash_map (key is a node pointer, data is a node handler,
	//                    NodeHasher is the hash function to be used,
	//                    eqnode is used to break ties when hash values are equal)
	unordered_set<MyNode*, MyNode::NodeHasher, MyNode::eqnode> nodes;

	MyNode* root = new MyNode(root_location, 0);
	root->open_handle = heap.push(root);  // add root to heap
	nodes.insert(root);       // add root to hash_table (nodes)
	while (!heap.empty())
	{
		MyNode* curr = heap.top();
		heap.pop();
		auto neighbours = children_vertices(curr->loc);
		for (auto next_loc : neighbours)
		{
			int next_g_val = (int)curr->g_val + 1;
			MyNode* next = new MyNode(next_loc, next_g_val);
			auto it = nodes.find(next);
			if (it == nodes.end()) {  // add the newly generated node to heap and hash table
				next->open_handle = heap.push(next);
				nodes.insert(next);
			}
			else {  // update existing node's g_val if needed (only in the heap)
				delete(next);  // not needed anymore -- we already generated it before
				MyNode* existing_next = *it;
				if (existing_next->g_val > next_g_val) {
					existing_next->g_val = next_g_val;
					heap.update((*it)->open_handle);
				}
			}
		}
	}
	// iterate over all nodes
	heuristics.resize(rows * cols, INT_MAX);
	for (auto it = nodes.begin(); it != nodes.end(); it++)
	{
		MyNode* s = *it;
		heuristics[s->loc] = (int)s->g_val;
		delete (s);
	}
	nodes.clear();
	heap.clear();
}
