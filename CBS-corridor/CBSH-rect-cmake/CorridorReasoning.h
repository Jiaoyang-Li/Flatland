#pragma once
#include "common.h"
#include "ConstraintTable.h"
#include "LLNode.h"
#include "map_loader.h"
#include "flat_map_loader.h"
#include "compute_heuristic.h"
#include "SingleAgentICBS.h"
#include <vector>

typedef std::tuple<int, int, int> CottidorTable_Key; // endpoint1, endpoint2, length
typedef std::unordered_map<CottidorTable_Key, int, three_tuple_hash> CorridorTable; // value = length of the bypass


bool isConstrained(int curr_id, int next_id, int next_timestep, const std::vector< std::list< std::pair<int, int> > >* cons);

int getDegree(int loc, const bool*map, int num_col, int map_size);


int getCorridorLength(const std::vector<PathEntry>& path, int t_start, int loc_end, std::pair<int, int>& edge);


bool validMove(int curr, int next, int map_cols, int map_size);

int getMahattanDistance(int loc1, int loc2, int map_cols);

template<class Map>
class CorridorReasoning {
public:
	int getEnteringTime(const std::vector<PathEntry>& path, const std::vector<PathEntry>& path2, int t,
		 Map* map);
	int getExitTime(const std::vector<PathEntry>& path, const std::vector<PathEntry>& path2, int t,
		Map* map);
	int getBypassLength(int start, int end, std::pair<int, int> blocked,  Map* my_map, int num_col, int map_size, int start_heading = -1);
	int getBypassLength(int start, int end, std::pair<int, int> blocked,  Map* my_map, int num_col, int map_size, ConstraintTable& constraint_table, int upper_bound, std::vector<hvals> restable, int start_heading = -1);
	int getBypassLength(int start, int end, int start_heading, int end_heading, std::pair<int, int> blocked, Map* my_map, int num_col, int map_size, ConstraintTable& constraint_table, int upper_bound, PathEntry& start_entry, float speed =1.0);
};


