/*
 * @author: Team An_old_driver
 * @created: 09-2020
 * Copyright (c) 2020 The University of Southern California. All Rights Reserved.
 * Copyrights licensed under an Academic/non-profit use license.
 * See the accompanying LICENSE file for terms.
*/

#pragma once

#include <tuple>
#include <boost/heap/fibonacci_heap.hpp>
#include <set>
#include <iostream>
#include <unordered_map>
#include <list>
#include <vector>
#include <unordered_set>
#include <memory>  // std::shared_ptr
#include <chrono>


#define MAX_COST INT_MAX/2
enum conflict_type { CORRIDOR2, CORRIDOR4, STANDARD,CHASING,CORRIDOR,START, TYPE_COUNT };
enum conflict_priority { CARDINAL, SEMI, NON, UNKNOWN, PRIORITY_COUNT };

enum constraint_type { RANGE, VERTEX, EDGE, CONSTRAINT_COUNT };
typedef std::tuple<int, int, int, constraint_type> Constraint;
typedef std::list<std::shared_ptr<std::tuple<int, int, int, int, int,int>>> OldConfList;

enum constraint_strategy { CBS, ICBS, CBSH, STRATEGY_COUNT };

using std::pair;
using std::make_pair;
using std::unordered_map;
using std::list;
using std::vector;
using std::tuple;
using std::set;

namespace N
{
	template<typename T>
	void get(T); //no need to provide definition
				 // as far as enabling ADL is concerned!
}


struct PathEntry
{

	int location = -1;
	int heading = -1;
	bool single = false;
	bool malfunction = false;
	int malfunction_left = 0;
	float position_fraction = 0.0;
	int exit_heading = -1;
	int exit_loc = -1;
	//PathEntry(int loc = -1) { location = loc; single = false; }
	//std::list<int> locations; // all possible locations at the same time step (i.e., mdd nodes at the same time step)
	OldConfList* conflist=nullptr;
};

struct options {
    bool asymmetry_constraint=false;
    int debug = 0;
    bool ignore_t0=false;
    bool shortBarrier=false;
    bool flippedRec=false;
};

struct Transition {
	int location = -1;
	int heading = -2;
	float position_fraction=0.0;
	int exit_loc=-1;
	int exit_heading = -1;
    Transition()=default;
    Transition(int location, int heading): location(location), heading(heading) {}
    Transition(int location, int heading, float position_fraction, int exit_loc, int exit_heading):
        location(location), heading(heading), position_fraction(position_fraction),
        exit_loc(exit_loc), exit_heading(exit_heading) {}
};

typedef pair<int, int> Interval; // [t_min, t_max)


typedef std::vector<PathEntry> Path;

// A hash function used to hash a pair of any kind
struct hash_pair {
    template <class T1, class T2>
    size_t operator()(const pair<T1, T2>& p) const
    {
        auto hash1 = std::hash<T1>{}(p.first);
        auto hash2 = std::hash<T2>{}(p.second);
        return hash1 ^ hash2;
    }
};

 //Only for three-tuples of std::hash-able types for simplicity.
 //You can of course template this struct to allow other hash functions
struct three_tuple_hash {
	template <class T1, class T2, class T3>
	std::size_t operator () (const std::tuple<T1, T2, T3> &p) const {
		using N::get;

		auto h1 = std::hash<T1>{}(get<0>(p));
		auto h2 = std::hash<T2>{}(get<1>(p));
		auto h3 = std::hash<T3>{}(get<2>(p));
		// Mainly for demonstration purposes, i.e. works but is overly simple
		// In the real world, use sth. like boost.hash_combine
		return h1 ^ h2 ^ h3;
	}
};

int getLocation(const std::vector<PathEntry>& path, int timestep);

int getMahattanDistance(int loc1_x, int loc1_y, int loc2_x, int loc2_y);
