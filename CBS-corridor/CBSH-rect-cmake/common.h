#pragma once

#include <tuple>
#include <boost/heap/fibonacci_heap.hpp>
#include <iostream>
#include <unordered_map>
#include <list>
#include <vector>
#include <unordered_set>
#include <memory>  // std::shared_ptr
enum conflict_type { TARGET, CORRIDOR2, CORRIDOR4, RECTANGLE, STANDARD, TYPE_COUNT };
enum conflict_priority { CARDINAL, SEMI, NON, UNKNOWN, PRIORITY_COUNT };

enum constraint_type { LENGTH, RANGE, BARRIER, VERTEX, EDGE, CONSTRAINT_COUNT };
typedef std::tuple<int, int, int, constraint_type> Constraint;
typedef std::list<std::shared_ptr<std::tuple<int, int, int, int, int,int>>> OldConfList;

enum constraint_strategy { CBS, ICBS, CBSH, CBSH_CR, CBSH_R, CBSH_RM, CBSH_GR, STRATEGY_COUNT };

using std::pair;
using std::make_pair;
using std::unordered_map;
using std::list;
using std::vector;
using std::tuple;

namespace N
{
	template<typename T>
	void get(T); //no need to provide definition
				 // as far as enabling ADL is concerned!
}

struct PathEntry
{

	int location;
	int heading;
	bool single;
	int actionToHere;
	bool malfunction = false;
	int malfunction_left = 0;
	int next_malfunction = 0;
	float position_fraction;
	int exit_heading;
	int exit_loc;
	PathEntry(int loc = -1) { location = loc; single = false; }
	std::list<int> locations; // all possible locations at the same time step
	OldConfList* conflist=NULL;
};

struct Transition {
	int first;//location
	int second;//heading
	float position_fraction=0.0;
	int exit_loc=-1;
	int exit_heading = -1;
    bool turn_active = false;
};

struct MDDPath {
	vector<std::unordered_set<int>> levels;
	void print() {
		for (int l = 0; l < levels.size(); l++) {
			std::unordered_set<int>::iterator it;
			std::cout << "level " << l << ": ";
			for (it = levels[l].begin(); it != levels[l].end(); ++it) {
				std::cout << *it << ",";
			}
			std::cout << std::endl;
		}
	}
	void print(int col) {
		for (int l = 0; l < levels.size(); l++) {
			std::unordered_set<int>::iterator it;
			std::cout << "level " << l << ": ";
			for (it = levels[l].begin(); it != levels[l].end(); ++it) {
				std::cout <<"("<< *it/col << "," << *it%col <<") ";
			}
			std::cout << std::endl;
		}
	}
};



typedef std::vector<PathEntry> Path;


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
