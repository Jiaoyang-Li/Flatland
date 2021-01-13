/*
 * @author: Team An_old_driver
 * @created: 09-2020
 * Copyright (c) 2020 The University of Southern California. All Rights Reserved.
 * Copyrights licensed under an Academic/non-profit use license.
 * See the accompanying LICENSE file for terms.
*/
#pragma once
#include <climits>
#include "common.h"

struct CT_entry{
    int timestep;
    int agent_id;
    int to = -1;

    CT_entry(int t, int agent_id, int to){
        this->timestep = t;
        this->agent_id = agent_id;
        this->to = to;
    }
};

class ConstraintTable
{
public:
	int length_max = INT_MAX;
	bool use_list = false;
    int num_of_agents = 10000;
	void reset()
	{
	    auto map_size = CT_paths.size(); CT_paths.clear(); CT_paths.resize(map_size);
        latest_conatraints.resize(map_size, 0);
    }
	bool insert_path(int agent_id, const Path& path);
    void delete_path(int agent_id, const Path& path);
    bool insert_path_list(int agent_id, const Path& path);
    void delete_path_list(int agent_id, const Path& path);
	bool is_constrained(int agent_id, int loc, int t, int pre_loc = -2) const;
    bool is_constrained_list(int agent_id, int loc, int t, int pre_loc = -2) const;
	bool blocked(int loc, int t) const;
	// bool is_good_malfunction_location(int loc, int t);
    void get_agents(set<int>& conflicting_agents, int loc) const;
    void get_agents(list< pair<int, int> >& agents, int excluded_agent,
            const tuple<int, int, int>& loc_timeinterval_pair) const;
    void get_agents(set<int>& conflicting_agents, int groupsize, int loc) const;
	void get_conflicting_agents(int agent_id, set<int>& conflicting_agents, int loc, int t) const;
    int get_latest_constrained_timestep(int loc) const { return latest_conatraints[loc]; }
    void init(size_t map_size)
    {
        latest_conatraints.resize(map_size, 0);
        if (use_list)
            CT_paths_list.resize(map_size);
        else
            CT_paths.resize(map_size);
    }
private:
    vector<int> latest_conatraints; // latest constraint at each location
    vector< vector<int> > CT_paths; // this stores the already planned paths, the value is the id of the agent
    vector< list<CT_entry> > CT_paths_list; // this stores the already planned paths, the value is the id of the agent

};

