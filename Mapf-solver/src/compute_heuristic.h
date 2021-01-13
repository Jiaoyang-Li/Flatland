/*
 * @author: Team An_old_driver
 * @created: 09-2020
 * Copyright (c) 2020 The University of Southern California. All Rights Reserved.
 * Copyrights licensed under an Academic/non-profit use license.
 * See the accompanying LICENSE file for terms.
*/
#pragma once

#include <vector>
#include <utility>
#include <stdlib.h>
#include "map_loader.h"
#include "flat_map_loader.h"

#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>

using namespace std;

struct hvals {
	int heading[4] = {MAX_COST, MAX_COST, MAX_COST, MAX_COST};
	int get_hval(int direction) const {
		if (direction >= 0) {
			return heading[direction];
		}
		else {
			return MAX_COST;
		}
	}
};
template<class Map>
class ComputeHeuristic 
{
 public:
  int start_location;
  int goal_location;
  int start_heading;
  const Map* ml;
  int map_rows;
  int map_cols;
  ComputeHeuristic();
  ComputeHeuristic(int start_location, int goal_location, const Map* ml0, int start_heading = 4);
 
 // bool validMove(int curr, int next) const;

 void getHVals(vector<hvals>& res,int limit = INT_MAX);


  ~ComputeHeuristic();

};


