#pragma once
#include "SinglePlanningFlat.h"
#include <boost/python.hpp>
namespace p = boost::python;

class CPR
{
public:
    CPR(AgentsLoader& al, FlatlandLoader& ml, const options& options1, float time_limit);
    void getNextLoc(vector<int>& to_go);
    void update(p::list agent_location);
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


