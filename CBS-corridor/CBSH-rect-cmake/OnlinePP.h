#pragma once
#include "SinglePlanning.h"


class OnlinePP {
public:
    list<int> to_be_planned_group; // a list of start locations
    list<Agent*> to_be_planned_agents; // a list of start locations

    OnlinePP(AgentsLoader& al, FlatlandLoader& ml, const options& options1, float time_limit);
    bool planPaths(float time_limit);
    void updateToBePlanedAgents(int max_num_agents);

private:
    bool replan = false;
    float runtime = 0;
    AgentsLoader& al;
    FlatlandLoader& ml;
    options options1;
    vector< list<Agent*> > unplanned_agents; // key: start location; value: list of agents

    void addAgentPath(int agent, const Path& path);
};

