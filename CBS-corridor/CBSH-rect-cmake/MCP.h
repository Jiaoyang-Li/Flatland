#pragma once
#include "agents_loader.h"
#include <boost/python.hpp>
namespace p = boost::python;

class MCP {
public:
    vector<int> to_go;

    void getNextLoc(p::list agent_location, int timestep);
    void update(p::list agent_location, p::dict agent_action);
    void build(const AgentsLoader* al, const FlatlandLoader* ml, options options1, int timestep = 0);
    void clear(void)
    {
        mcp.clear();
        agent_time.clear();
        appear_time.clear();
        no_wait_time.clear();
    };
    void printAll(void);
    void print(int loc);
    void printAgentTime(void);
    void printAgentNoWaitTime(void);
    void simulate(vector<Path>& paths, int timestep) const;
    int getEstimatedCost(int timestep) const;
private:
    const AgentsLoader* al = nullptr;
    const FlatlandLoader* ml = nullptr;
    options options1;

    typedef list<tuple<int, int>> Occupy;
    vector<Occupy> mcp;
    vector<int> agent_time;

    vector<int> appear_time;
    vector<vector<int>> no_wait_time;
};

