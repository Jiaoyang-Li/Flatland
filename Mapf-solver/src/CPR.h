/*
 * @author: Team An_old_driver
 * @created: 09-2020
 * Copyright (c) 2020 The University of Southern California. All Rights Reserved.
 * Copyrights licensed under an Academic/non-profit use license.
 * See the accompanying LICENSE file for terms.
*/

#pragma once
#include "SinglePlanning.h"
#include <boost/python.hpp>
namespace p = boost::python;

class CPR
{
public:
    vector<int> to_go;

    CPR(AgentsLoader& al, FlatlandLoader& ml, const options& options1, float time_limit);
    void getNextLoc();
    void update();
    void planPaths(float time_limit);

private:
    bool replan = false;
    high_resolution_clock::time_point start_time;
    float runtime = 0;
    AgentsLoader& al;
    FlatlandLoader& ml;
    options options1;
    list<Agent*> unplanned_agents;
    vector<int> agent_steps;
};


