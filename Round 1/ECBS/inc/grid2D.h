#pragma once
#include "common.h"

class Grid2D 
{
 public:
	 vector<int> start_locations;
	 vector<int> goal_locations;
	 vector< vector<int> > heuristics; // [agent_id][loc]

	 list<int> adjacent_vertices(int vertex_id) const;

	 inline size_t map_size() const { return rows * cols; }


	 Grid2D() {}
	 bool load_map(std::string fname); // load map from file
	 bool load_agents(std::string fname); // load map from file

	 void preprocessing_heuristics();

 private:
  vector<bool> my_map;
  int rows;
  int cols;
  enum valid_moves_t { NORTH, EAST, SOUTH, WEST, WAIT_MOVE, MOVE_COUNT };  // MOVE_COUNT is the enum's size
  int moves_offset[5];

  inline int linearize_coordinate(int row, int col) const { return (this->cols * row + col); }
  inline int row_coordinate(int id) const { return id / this->cols; }
  inline int col_coordinate(int id) const { return id % this->cols; }
  void compute_heuristics(int goal_location, vector<int>& heuristics);

};
