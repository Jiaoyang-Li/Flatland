/*
 * @author: Team An_old_driver
 * @created: 09-2020
 * Copyright (c) 2020 The University of Southern California. All Rights Reserved.
 * Copyrights licensed under an Academic/non-profit use license.
 * See the accompanying LICENSE file for terms.
*/
#include <boost/heap/fibonacci_heap.hpp>
#include "ConstraintTable.h"
#include "compute_heuristic.h"
#include <iostream>
#include "LLNode.h"
#include <limits.h>

using std::cout;
using std::endl;
using boost::heap::fibonacci_heap;


template<class Map>
ComputeHeuristic<Map>::ComputeHeuristic() {}

template<class Map>
ComputeHeuristic<Map>::ComputeHeuristic(int start_location, int goal_location, const Map* ml0, int start_heading):
    ml(ml0) {
	map_rows = ml->rows;
	map_cols = ml->cols;
	this->start_location = start_location;
	this->goal_location = goal_location;
	this->start_heading = start_heading;

}



template<class Map>
void ComputeHeuristic<Map>::getHVals(vector<hvals>& res,int limit)
{
	res.resize(map_rows * map_cols);
	vector<bool> visited(map_rows * map_cols * 4, false);
	std::list<LLNode*> queue;
    for (int heading = 0; heading < 4; heading++) {
        auto root = new LLNode(goal_location, 0, 0, nullptr, 0);
        root->heading = heading;
        queue.push_front(root);
        res[goal_location].heading[heading] = 0;
        visited[goal_location * 4 + heading] = true;
    }
	
	while (!queue.empty()) {
		LLNode* curr = queue.back();
		queue.pop_back();
		list<Transition> transitions;
		ml->get_transitions(transitions, curr->loc,curr->heading,true);
		for (const auto& move:transitions)
		{
            assert(move.heading < 4); // no wait
			int next_loc = move.location;
			int next_g_val = curr->g_val + 1;
            int rheading = (move.heading + 2) % 4; // reverse heading

            if (res[curr->loc].heading[rheading] > curr->g_val)
            {
                res[curr->loc].heading[rheading] = curr->g_val;
            }
            if (!visited[next_loc * 4 + move.heading])
            {
                visited[next_loc * 4 + move.heading] = true;
                auto next = new LLNode(next_loc, next_g_val, 0, nullptr, 0);
                next->heading = move.heading;
                queue.push_front(next);
            }
		}
        delete curr;
	}
}

template<class Map>
ComputeHeuristic<Map>::~ComputeHeuristic() {
}

template class ComputeHeuristic<MapLoader>;
template class ComputeHeuristic<FlatlandLoader>;

