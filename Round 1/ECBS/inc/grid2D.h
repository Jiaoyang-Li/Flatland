#pragma once
#include "common.h"

class Grid2D 
{
 public:
	 vector<int> start_ids;
	 vector<int> goal_ids;
	 vector< vector<int> > heuristics; // [agent_id][loc]

	 list<int> children_vertices(int vertex_id) const;

     bool is_node_conflict(int id_0, int id_1) const {return id_0 == id_1; }
     bool is_edge_conflict(int from_0, int to_0, int from_1, int to_1) const
     {
         return from_0 == to_1 && from_1 == to_0;
     }

     int get_location(int id) const {return id; }
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
