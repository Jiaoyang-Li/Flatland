#pragma once
#include "common.h"

class FlatlandMap
{
 public:
  vector<int> start_ids;
  vector<list<int>> goal_ids;
  vector<int> goal_locations;
  vector<int> r_velocities; // reciprocals of velocities, how many timesteps for train to cross an edge
  vector< vector<int> > heuristics; // [agent_id][id]

  inline size_t map_size() const { return num_vertexes; }

  list<int> children_vertices(int vertex_id) const;

  int get_location(int id) const {return node2loc[id]; }
  bool is_node_conflict(int id_0, int id_1) const;
  bool is_edge_conflict(int from_0, int to_0, int from_1, int to_1) const;


  FlatlandMap() {}
  bool load_map(std::string fname); // load map from file
  bool load_agents(std::string fname); // load map from file

  void preprocessing_heuristics();

 private:
  // vector<bool> my_map;
  bool allowed_wait = true;


  vector<int> node2loc;

  int num_vertexes;
  std::vector<std::vector<int>> edges;
  std::vector<std::vector<int>> edges_r;// reversed edges used to compute heuristics

  // enum valid_moves_t { NORTH, EAST, SOUTH, WEST, WAIT_MOVE, MOVE_COUNT };  // MOVE_COUNT is the enum's size
  // int moves_offset[5];

  /* inline int linearize_coordinate(int row, int col) const { return (this->cols * row + col); } */
  /* inline int row_coordinate(int id) const { return id / this->cols; } */
  /* inline int col_coordinate(int id) const { return id % this->cols; } */
  void compute_heuristics(int agent_id);

  list<int> adjacent_vertices_r(int vertex_id) const;
  list<int> adjacent_vertices(int vertex_id) const;
};
 
