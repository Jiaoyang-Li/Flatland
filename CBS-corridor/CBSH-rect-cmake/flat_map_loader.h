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
	int highways[4] = {0, 0, 0, 0}; // the move on a given direction i is allowed only when highways[i] >= 0
};

class FlatlandLoader:public MapLoader {
public:
	FlatlandLoader(boost::python::object rail1, int rows, int cols);
	railCell get_full_cell(int location);
	FlatlandLoader();
	int getDegree(int loc) const;
    int getDegree(int loc,int heading) const;
    float getMalfunctionRate() const {return this->malfunction_rate;}
    float setMalfunctionRate(float rate){this->malfunction_rate = rate;}

	boost::python::object rail;
	railCell* railMap;
	void get_transitions(list<Transition>& transition, int location, int heading = -1, bool noWait=false) const;
	void get_exits(list<Transition>& transition, int location, int heading = -1,float speed=1.0, bool noWait = false) const;
	~FlatlandLoader();
protected:
	float blockRate;
	float malfunction_rate = 0;
};

