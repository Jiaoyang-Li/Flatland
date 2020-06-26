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


vector<Transition> FlatlandLoader::get_transitions(int location, int heading, bool noWait) const {
	vector<Transition> transitions;
	int cell_transition = railMap[location].transitions;
	int bits = (cell_transition >> ((3 - heading) * 4));
	int moves[4] = { (bits >> 3) & 1, (bits >> 2) & 1, (bits >> 1) & 1, (bits) & 1 };

	for (int i = 0; i < 4; i++) {
		int moveable = moves[i];

		if (moveable == 1) {
			Transition move;

			move.first = location + this->moves_offset[i];
			move.second = i;
			transitions.push_back(move);

		}
	}
	Transition wait;
	wait.first = location;
	wait.second = 4;
	if (!noWait)
		transitions.push_back(wait);

	return transitions;
}

vector<Transition> FlatlandLoader::get_exits(int location, int heading,float speed, bool noWait) const {
	vector<Transition> transitions;
	int cell_transition = railMap[location].transitions;
	int bits = (cell_transition >> ((3 - heading) * 4));
	int moves[4] = { (bits >> 3) & 1, (bits >> 2) & 1, (bits >> 1) & 1, (bits) & 1 };

	for (int i = 0; i < 4; i++) {
		int moveable = moves[i];

		if (moveable == 1) {
			Transition move;

			move.first = location;
			move.exit_loc = location + this->moves_offset[i];
			move.second = heading;
			move.exit_heading = i;
			move.position_fraction = 0.0 + speed;
			transitions.push_back(move);

		}
	}
	Transition wait;
	wait.first = location;
	wait.second = 4;
	wait.position_fraction = 0.0;
	if (!noWait)
		transitions.push_back(wait);

	return transitions;
}

int FlatlandLoader::getDegree(int loc) {
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

FlatlandLoader::~FlatlandLoader(){
	delete[] this->my_map;
	delete[] this->moves_offset;
	delete[] this->railMap;
}

