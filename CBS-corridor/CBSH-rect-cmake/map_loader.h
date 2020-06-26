// Load's a 2D map.
#pragma once

#include <string>
#include <vector>
#include "common.h"

using namespace std;




class MapLoader 
{
 public:
  bool* my_map;
  int rows;
  int cols;


  int start_loc;
  int goal_loc;

  enum valid_moves_t { NORTH, EAST, SOUTH, WEST, WAIT_MOVE, MOVE_COUNT };  // MOVE_COUNT is the enum's size
  int *moves_offset;
  bool validMove(int curr, int next) const;

  MapLoader(std::string fname); // load map from file
  MapLoader(int rows, int cols); // initialize new [rows x cols] empty map
  MapLoader();
  vector<Transition> get_transitions(int loc, int heading, int noWait) const;
  vector<Transition> get_exits(int loc, int heading,float speed, int noWait) const;

  bool getLoc(int loc) ;
  inline bool is_blocked (int row, int col) const { return my_map[row * this->cols + col]; }
  inline bool is_blocked (int loc) const { return my_map[loc]; }
  inline size_t map_size() const { return rows * cols; }
  void printMap ();
  void printMap (char* mapChar);
  char* mapToChar();
  bool* get_map () const; // return a deep-copy of my_map
  inline int linearize_coordinate(int row, int col) const { return ( this->cols * row + col); }
  inline int row_coordinate(int id) const { return id / this->cols; }
  inline int col_coordinate(int id) const { return id % this->cols; }
  void printPath (std::vector<int> path);
  void saveToFile(std::string fname);
  int getDegree(int loc);
  ~MapLoader();
};