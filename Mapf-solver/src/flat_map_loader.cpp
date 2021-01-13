/*
 * @author: Team An_old_driver
 * @created: 09-2020
 * Copyright (c) 2020 The University of Southern California. All Rights Reserved.
 * Copyrights licensed under an Academic/non-profit use license.
 * See the accompanying LICENSE file for terms.
*/

#include "flat_map_loader.h"
#include <iostream>
#include <fstream>
#include<boost/tokenizer.hpp>
#include <cstring>
#include <bitset>
#include <boost/python.hpp>




namespace p = boost::python;

using namespace boost;
using namespace std;

FlatlandLoader::FlatlandLoader() {  }

FlatlandLoader::FlatlandLoader(p::object rail1, int rows, int cols) {
	this->rail = rail1;
	this->rows = rows;
	this->cols = cols;
	this->my_map = new bool[false];

	// Possible moves [WAIT, NORTH, EAST, SOUTH, WEST]
	moves_offset = new int[MapLoader::MOVE_COUNT];
	moves_offset[MapLoader::valid_moves_t::WAIT_MOVE] = 0;
	moves_offset[MapLoader::valid_moves_t::NORTH] = -cols;
	moves_offset[MapLoader::valid_moves_t::EAST] = 1;
	moves_offset[MapLoader::valid_moves_t::SOUTH] = cols;
	moves_offset[MapLoader::valid_moves_t::WEST] = -1;

	railMap = new railCell[rows*cols];
	int notRailCount = 0;
	for (int loc = 0; loc < rows*cols; loc++) {
		railMap[loc] = get_full_cell(loc);
		if (railMap[loc].transitions == 0)
			notRailCount += 1;

	}
	blockRate = notRailCount / (rows*cols);

};


railCell FlatlandLoader::get_full_cell(int location) {
	railCell full_cell;
	p::object trans = rail.attr("get_full_transitions")(row_coordinate(location), col_coordinate(location));
	p::object is_simple_turn = rail.attr("is_simple_turn")(p::make_tuple(row_coordinate(location), col_coordinate(location)));
	p::object is_dead_end = rail.attr("is_dead_end")(p::make_tuple(row_coordinate(location), col_coordinate(location)));
	p::long_ transLong(trans);
	p::object True(true);
	//p::object False(false);
	//cout << location << endl;
	//if (True == is_dead_end)
	//	cout << "true == is_dead_end" << endl;
	//else
	//	cout << "true != is_dead_end" << endl;

	//if (False == is_dead_end)
	//	cout << "false == is_dead_end" << endl;
	//else
	//	cout << "false != is_dead_end" << endl;

	int transInt = p::extract<int>(transLong);
	full_cell.transitions = transInt;
	full_cell.isDeadEnd = is_dead_end==True;
	full_cell.isTurn =is_simple_turn==True;
	//cout << location << " done" << endl;

	return full_cell;
}


void FlatlandLoader::get_transitions(list<Transition>& transitions, int location, int heading, bool noWait) const {
	int cell_transition = railMap[location].transitions;
	int bits = (cell_transition >> ((3 - heading) * 4));
	int moves[4] = { (bits >> 3) & 1, (bits >> 2) & 1, (bits >> 1) & 1, (bits) & 1 };

	for (int i = 0; i < 4; i++) {
		if (moves[i] == 1 && railMap[location + this->moves_offset[i]].highways[(i + 2) % 4] == 0) {
			transitions.emplace_back(location + this->moves_offset[i], i); // location + heading
		}
	}

	if (!noWait)
    {
        transitions.emplace_back(location, heading); // location + heading
    }
}

void FlatlandLoader::get_exits(list<Transition>& transitions, int location, int heading,float speed, bool noWait) const {
	int cell_transition = railMap[location].transitions;
	int bits = (cell_transition >> ((3 - heading) * 4));
	int moves[4] = { (bits >> 3) & 1, (bits >> 2) & 1, (bits >> 1) & 1, (bits) & 1 };

	for (int i = 0; i < 4; i++) {
		if (moves[i] == 1 && railMap[location].highways[i] >= 0) {
			Transition move;

			move.location = location;
			move.exit_loc = location + this->moves_offset[i];
			move.heading = heading;
			move.exit_heading = i;
			move.position_fraction = 0.0 + speed;
			transitions.push_back(move);

		}
	}
	if (!noWait)
    {
        Transition wait;
        wait.location = location;
        wait.heading = heading;
        wait.position_fraction = 0.0;
        transitions.push_back(wait);
    }
}

int FlatlandLoader::getHeading(int from, int to) const
{
    for (int i = 0; i < 4; i++)
        if (from + this->moves_offset[i] == to)
            return i;
    return -1;
}

int FlatlandLoader::getDegree(int loc) const{
	if (loc < 0 || loc >= map_size())
		return -1;
	std::unordered_set<int> possibleMoves;
	for (int heading = 0; heading < 4; heading++) {
		int cell_transition = railMap[loc].transitions;
		int bits = (cell_transition >> ((3 - heading) * 4));
		int moves[4] = { (bits >> 3) & 1, (bits >> 2) & 1, (bits >> 1) & 1, (bits) & 1 };
		for (int direction = 0; direction < 4;direction++) {
			if (moves[direction] == 1) {
				possibleMoves.insert(direction);
			}
		}
	}
	return possibleMoves.size();
}

int FlatlandLoader::getDegree(int loc,int heading) const{
    if (loc < 0 || loc >= map_size())
        return -1;
    std::unordered_set<int> possibleMoves;
    int cell_transition = railMap[loc].transitions;
    int bits = (cell_transition >> ((3 - heading) * 4));
    int moves[4] = { (bits >> 3) & 1, (bits >> 2) & 1, (bits >> 1) & 1, (bits) & 1 };
    for (int direction = 0; direction < 4;direction++) {
        if (moves[direction] == 1) {
            possibleMoves.insert(direction);
        }

    }
    return possibleMoves.size();
}

FlatlandLoader::~FlatlandLoader(){
    delete[] this->railMap;
}

