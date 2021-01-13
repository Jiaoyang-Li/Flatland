/*
 * @author: Team An_old_driver
 * @created: 09-2020
 * Copyright (c) 2020 The University of Southern California. All Rights Reserved.
 * Copyrights licensed under an Academic/non-profit use license.
 * See the accompanying LICENSE file for terms.
*/

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
    int getHeading(int from, int to) const;
	boost::python::object rail;
	railCell* railMap;
	void get_transitions(list<Transition>& transition, int location, int heading = -1, bool noWait=false) const;
	void get_exits(list<Transition>& transition, int location, int heading = -1,float speed=1.0, bool noWait = false) const;
	~FlatlandLoader();
protected:
	float blockRate;
};

