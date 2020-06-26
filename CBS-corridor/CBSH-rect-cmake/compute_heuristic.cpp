#include <boost/heap/fibonacci_heap.hpp>
#include "ConstraintTable.h"
#include "compute_heuristic.h"
#include <google/dense_hash_map>
#include <iostream>
#include "LLNode.h"
#include <limits.h>


using google::dense_hash_map;      // namespace where class lives by default
using std::cout;
using std::endl;
using boost::heap::fibonacci_heap;

//FlatlandComputeHeuristic::FlatlandComputeHeuristic(int start_location, int goal_location, FlatlandLoader* fl0, int start_heading)  {
//	fl = fl0;
//	map_rows = fl->rows;
//	map_cols = fl->cols;
//	this->start_location = start_location;
//	this->goal_location = goal_location;
//	this->start_heading = start_heading;
//
//}
//
//vector<pair<int, int>> FlatlandComputeHeuristic::get_transitions(int loc, int heading, int noWait ) {
//	vector<pair<int, int>> transitions = fl->get_transitions(loc, heading, noWait);
//	return transitions;
//}

template<class Map>
ComputeHeuristic<Map>::ComputeHeuristic() {}

template<class Map>
ComputeHeuristic<Map>::ComputeHeuristic(int start_location, int goal_location, Map* ml0, int start_heading) {
	ml = ml0;
	map_rows = ml->rows;
	map_cols = ml->cols;
	this->start_location = start_location;
	this->goal_location = goal_location;
	this->start_heading = start_heading;

}



template<class Map>
void ComputeHeuristic<Map>::getHVals(vector<hvals>& res,int limit)
{
	size_t root_location = goal_location;
	res.resize(map_rows * map_cols);
		
	// generate a heap that can save nodes (and a open_handle)
	boost::heap::fibonacci_heap< LLNode*, boost::heap::compare<LLNode::compare_node> > heap;
	boost::heap::fibonacci_heap< LLNode*, boost::heap::compare<LLNode::compare_node> >::handle_type open_handle;
	// generate hash_map (key is a node pointer, data is a node handler,
	//                    NodeHasher is the hash function to be used,
	//                    eqnode is used to break ties when hash values are equal)
	dense_hash_map<LLNode*, fibonacci_heap<LLNode*, boost::heap::compare<LLNode::compare_node> >::handle_type, LLNode::NodeHasher, LLNode::eqnode> nodes;
	nodes.set_empty_key(NULL);
	dense_hash_map<LLNode*, fibonacci_heap<LLNode*, boost::heap::compare<LLNode::compare_node> >::handle_type, LLNode::NodeHasher, LLNode::eqnode>::iterator it; // will be used for find()

	if (start_heading == -1) {
		LLNode* root = new LLNode(root_location, 0, 0, NULL, 0);
		root->heading = start_heading;
		root->open_handle = heap.push(root);  // add root to heap
		nodes[root] = root->open_handle;       // add root to hash_table (nodes)
	}
	else {
		for (int heading = 0; heading < 4; heading++) {
			LLNode* root = new LLNode(root_location, 0, 0, NULL, 0);
			root->heading = heading;
			root->open_handle = heap.push(root);  // add root to heap
			nodes[root] = root->open_handle;
		}
	}
	
	while (!heap.empty()) {
		LLNode* curr = heap.top();
		heap.pop();

		vector<Transition> transitions = ml->get_transitions(curr->loc,curr->heading,true);
		for (const auto move:transitions)
		{
			int next_loc = move.first;
			int next_g_val = curr->g_val + 1;
			LLNode* next = new LLNode(next_loc, next_g_val, 0, NULL, 0);

			if (curr->heading == -1) //heading == -1 means no heading info
				next->heading = -1;
			else
				if (move.second == 4) //move == 4 means wait
					next->heading = curr->heading;
				else
					next->heading = move.second;

			curr->possible_next_heading.push_back(next->heading);


			it = nodes.find(next);
			if (it == nodes.end())
			{  // add the newly generated node to heap and hash table
				if (next_g_val > limit) {
					delete(next);
					continue;
				}
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
	// iterate over all nodes and populate the distances
	for (it = nodes.begin(); it != nodes.end(); it++) 
	{
		LLNode* s = (*it).first;
		if (s->heading == -1) {

			if (!res[s->loc].heading.count(-1)) {
				res[s->loc].heading[-1] = s->g_val;

			}
			else if (s->g_val < res[s->loc].heading[-1]) {
				res[s->loc].heading[-1] = s->g_val;


			}

		}
		else {
			
			if (s->possible_next_heading.size() > 0) {
				for (int& next_heading : s->possible_next_heading) {
					int heading = (next_heading + 2) % 4;

					if (!res[s->loc].heading.count(heading)) {
						res[s->loc].heading[heading] = s->g_val;
						
					}
					else if (s->g_val < res[s->loc].heading[heading]) {
						res[s->loc].heading[heading] = s->g_val;
						

					}

				}
			}

			int heading = (s->heading + 2) % 4;
			if (!res[s->loc].heading.count(heading)) {
				res[s->loc].heading[heading] = s->g_val;
				
			}
			else if (s->g_val < res[s->loc].heading[heading]) {
				res[s->loc].heading[heading] = s->g_val;

				
			}

			

		}

		delete (s);
	}
	nodes.clear();
	heap.clear();

}

template<class Map>
ComputeHeuristic<Map>::~ComputeHeuristic() {
}

template class ComputeHeuristic<MapLoader>;
template class ComputeHeuristic<FlatlandLoader>;

