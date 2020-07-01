// Load's a 2D map.
#pragma once

#include <string>
#include <vector>
#include "map_loader.h"
#include <boost/python.hpp>
#include "common.h"
namespace p = boost::python;
using namespace std;

struct railCell {
	int transitions;
	bool isTurn;
	bool isDeadEnd;
};

class FlatlandLoader:public MapLoader {
public:
	FlatlandLoader(boost::python::object rail1, int rows, int cols);
	railCell get_full_cell(int location);
	FlatlandLoader();
	int getDegree(int loc);
    int getDegree(int loc,int heading);

	boost::python::object rail;
	railCell* railMap;
	vector<Transition> get_transitions(int location, int heading = -1, bool noWait=false) const;
	vector<Transition> get_exits(int location, int heading = -1,float speed=1.0, bool noWait = false) const;
	~FlatlandLoader();
protected:
	float blockRate;
};

