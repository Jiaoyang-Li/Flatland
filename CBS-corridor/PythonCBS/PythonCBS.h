#include <iostream>
#include <string>
#include <vector>
#include <boost/python.hpp>
#include "flat_map_loader.h"


#include "ICBSSearch.h"


namespace p = boost::python;

template <class Map>
class PythonCBS {
public:
	PythonCBS(p::object railEnv1, std::string algo, int kRobust, int t, bool debug, float f_w, string corridor);

	p::list getResult();

	int defaultGroupSize = 16; // max number of agents in a group

	bool search();
	p::dict getResultDetail();
	void updateAgents(p::object railEnv1);
	void updateFw(float fw);
private:
	std::string algo;
	p::object railEnv;
	FlatlandLoader* ml;
	AgentsLoader* al;
	constraint_strategy s;
	options options1;
	int timeLimit;
	int kRobust;
	int max_malfunction;
	float f_w;
	// MultiMapICBSSearch<Map>* icbs = NULL;
	bool corridor2=false;
	bool corridor4=false;
	bool trainCorridor1 = false;
	bool trainCorridor2 = false;

	//stats
    std::clock_t start_time;
    double runtime;
    int HL_num_expanded = 0;
    int HL_num_generated = 0;
    int LL_num_expanded = 0;
    int LL_num_generated = 0;
    int num_rectangle = 0;
    int num_corridor2 = 0;
    int num_corridor4 = 0;

    void updateResults(const MultiMapICBSSearch<Map>& cbs)
    {
        runtime = (double)(std::clock() - start_time) / CLOCKS_PER_SEC;
        HL_num_expanded += cbs.HL_num_expanded;
        HL_num_generated += cbs.HL_num_generated;
        LL_num_expanded += cbs.LL_num_expanded;
        LL_num_generated += cbs.LL_num_generated;
        num_rectangle += cbs.num_rectangle;
        num_corridor2 += cbs.num_corridor2;
        num_corridor4 += cbs.num_corridor4;
    }

    bool hasConflicts(const vector<Path>& paths) const;
};


