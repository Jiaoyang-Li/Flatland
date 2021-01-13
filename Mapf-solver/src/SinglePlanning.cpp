/*
 * @author: Team An_old_driver
 * @created: 09-2020
 * Copyright (c) 2020 The University of Southern California. All Rights Reserved.
 * Copyrights licensed under an Academic/non-profit use license.
 * See the accompanying LICENSE file for terms.
*/

#include "SinglePlanning.h"
#include <stack>
#include <iostream>


void SinglePlanning::updatePath(LLNode* goal)
{
	path.resize(goal->g_val + 1);
	LLNode* curr = goal;

    for(int t = goal->g_val; t >= 0; t--)
	{

		path[t].location = curr->loc;
		path[t].heading = curr->heading;
		path[t].position_fraction = curr->position_fraction;
		path[t].malfunction_left = curr->malfunction_left;
		path[t].exit_heading = curr->exit_heading;
		path[t].exit_loc = curr->exit_loc;

        //assert(t == goal->g_val || ml.railMap[path[t].location].highways[(path[t+1].heading + 2) % 4] == 0);
		curr = curr->parent;
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// return true if a path found (and updates vector<int> path) or false if no path exists

bool SinglePlanning::search(bool flat)
{
    Time::time_point start_clock = Time::now();

    LL_num_expanded = 0;
    LL_num_generated = 0;

	hashtable_t::iterator it;  // will be used for find()


	 // generate start and add it to the OPEN list
	LLNode* start;
	if (agent.status == 0)
	    start = new LLNode(-1, 0, (1 + my_heuristic[start_location].get_hval(agent.heading)), nullptr, 0);
    else
        start = new LLNode(start_location, 0, my_heuristic[start_location].get_hval(agent.heading), nullptr, 0);
    int start_h_val = start->h_val;
    if (start->h_val < MAX_COST)
        start->h_val = start->h_val*f_w;
    start->heading = agent.heading;
    LL_num_generated++;
	start->open_handle = open_list.push(start);
	start->in_openlist = true;
	start->time_generated = 0;
	start->malfunction_left = agent.malfunction_left;

    
	start->position_fraction = agent.position_fraction;
	start->exit_heading = agent.exit_heading;
	if (start->exit_heading >= 0) {
		list<Transition> temp;
		ml.get_transitions(temp, start_location, start->heading, true);
		if (temp.size() == 1) {
			start->exit_loc = temp.front().location;
			start->exit_heading = temp.front().heading;
		}
		else
			start->exit_loc = start_location + ml.moves_offset[start->exit_heading];
	}



	allNodes_table.insert(start);


	int time_generated = 0;
	int time_check_count = 0;
    fsec runtime;
	/*for (int h = 0; h < my_heuristic.size();h++) {
		for (int heading = 0; heading<5;heading++)
			std::cout << "(" << h << ": heading:"<<-1 <<": "<< my_heuristic[h].heading[-1] << ")";
	}*/

	while (!open_list.empty())
	{
		if (LL_num_generated / 10000 > time_check_count && time_limit != 0) {
			runtime = Time::now() - start_clock;
			time_check_count = LL_num_generated / 10000;
			if (runtime.count() > time_limit) {
                releaseClosedListNodes();
				return false;
			}
		}

		LLNode* curr = open_list.top(); open_list.pop();

// 		assert(curr->h_val >=0);
//        cout<<"Current: "<<curr->h_val<< ": "<<curr->loc/num_col <<", "<<curr->loc%num_col << ", heading: "<<curr->heading<< endl;
//        for (auto heading : my_heuristic[curr->loc].heading){
//            cout << heading<< ",";
//        }
//        cout <<endl;

		curr->in_openlist = false;
        LL_num_expanded++;
		//cout << "focal size " << focal_list.size() << endl;
		//cout << "goal_location: " << goal_location << " curr time: " << curr->timestep << " length_min: " << constraintTable.length_min << endl;
		// check if the popped node is a goal
		if ((curr->loc == goal_location ) /*|| (curr->parent!= nullptr && curr->next_malfunction==0 && curr->parent->next_malfunction ==1)*/)
		{
			updatePath(curr);

			releaseClosedListNodes();
			return true;
			//}
		}
		

		list<Transition> transitions;
		if(flat){
            if(curr->loc == -1){
                transitions.emplace_back(start_location, curr->heading,
                        curr->position_fraction, curr->exit_loc, curr->exit_heading);
            }
            else
                ml.get_transitions(transitions, curr->loc, curr->heading, true);
		}
		else if (curr->malfunction_left>0 ){
            transitions.emplace_back(curr->loc, curr->heading, curr->position_fraction,
                    curr->exit_loc, curr->exit_heading);
		}
		else if(curr->loc == -1){
            transitions.emplace_back(-1, curr->heading, curr->position_fraction, curr->exit_loc, curr->exit_heading);
            transitions.emplace_back(start_location, curr->heading,
                                     curr->position_fraction, curr->exit_loc, curr->exit_heading);
            
        }
		else if ( curr->position_fraction>=1 && curr->exit_heading>=0 ){

                if (constraintTable.is_constrained(agent.agent_id, curr->exit_loc, curr->timestep+1,curr->loc)) {
                    transitions.emplace_back(curr->loc, curr->heading, curr->position_fraction,
                                             curr->exit_loc, curr->exit_heading);
                }
                else{
                    transitions.emplace_back(curr->exit_loc, curr->exit_heading);
                    transitions.back().position_fraction = 0;
                }
            }
		else
                ml.get_transitions(transitions, curr->loc, curr->heading, false);



		for (const auto& move : transitions)
		{
			int next_id = move.location;
			time_generated += 1;
			int next_timestep;
			if(flat)
                next_timestep = curr->timestep;
			else
			    next_timestep = curr->timestep + 1;


			if (flat || !constraintTable.is_constrained(agent.agent_id, next_id, next_timestep,curr->loc)) //&&
			{
			    int next_malfunction_left;
			    if (curr->malfunction_left>0)
			        next_malfunction_left = curr->malfunction_left -1;
                else
                    next_malfunction_left = 0;

				int next_g_val = curr->g_val + 1;
				int next_heading;

				if (curr->heading == -1) //heading == -1 means no heading info
					next_heading = -1;
				else
					next_heading = move.heading;
				float next_position_fraction = move.position_fraction;

                int next_h_val, next_show_time;
                if (curr->loc == -1 and next_id!=-1)
                    next_show_time = next_timestep;
                else
                    next_show_time = curr->show_time;

                if (next_id!=-1) {
                    next_h_val = my_heuristic[next_id].get_hval(next_heading);

                    if (next_g_val + next_h_val > constraintTable.length_max)
                        continue;

                    if (next_h_val < MAX_COST){
                        next_h_val = next_h_val*f_w;
                    }
                }
                else {
                    if (next_g_val + start_h_val> constraintTable.length_max)
                        continue;
                    next_h_val = curr->h_val;
                }
                assert(next_h_val >= 0 && next_h_val<=MAX_COST);




//                cout<<"Next: "<<next_h_val<<","<< next_id/num_col <<", "<<next_id%num_col<<", heading: "<<next_heading<< endl;
//                for (auto heading : my_heuristic[next_id].heading){
//                    cout << heading<< ",";
//                }
//                cout <<endl;


                // generate (maybe temporary) node
				auto next = new LLNode(next_id, next_g_val, next_h_val,	curr, next_timestep);
				next->heading = next_heading;
				next->time_generated = time_generated;
				next->position_fraction = next_position_fraction;
				next->exit_heading = move.exit_heading;
				next->exit_loc = move.exit_loc;
				next->show_time = next_show_time;
				next->malfunction_left = next_malfunction_left;

				// try to retrieve it from the hash table
				it = allNodes_table.find(next);
				if (it == allNodes_table.end())
				{

					next->open_handle = open_list.push(next);
					next->in_openlist = true;
                    LL_num_generated++;
					allNodes_table.insert(next);
				}
				else
				{  // update existing node's if needed (only in the open_list)
					delete(next);  // not needed anymore -- we already generated it before
					LLNode* existing_next = (*it);

					if (existing_next->in_openlist)
					{  // if its in the open list
						if (existing_next->g_val > next_g_val)
						{
							// update existing node
							existing_next->g_val = next_g_val;
							existing_next->h_val = next_h_val;
                            existing_next->timestep = next_timestep;
                            existing_next->show_time = next_show_time;
							existing_next->parent = curr;
							open_list.increase(existing_next->open_handle);  // increase because f-val improved
						}
					}
					else 
					{  // if its in the closed list (reopen)
						if (existing_next->g_val > next_g_val )
						{
							// if f-val decreased through this new path (or it remains the same and there's less internal conflicts)
							existing_next->g_val = next_g_val;
							existing_next->h_val = next_h_val;
                            existing_next->timestep = next_timestep;
                            existing_next->show_time = next_show_time;
                            existing_next->parent = curr;
							existing_next->open_handle = open_list.push(existing_next);
							existing_next->in_openlist = true;

						}
					}  // end update a node in closed list
				}  // end update an existing node
			}// end if case forthe move is legal
		}  // end for loop that generates successors
	}  // end while loop

	  // no path found
	releaseClosedListNodes();
	return false;
}




SinglePlanning::SinglePlanning(const FlatlandLoader& ml, AgentsLoader& al, double f_w, float time_limit, options option):
    ml(ml),al(al), f_w(f_w), time_limit(time_limit),
    my_heuristic(*al.agents[0]->heuristics),constraintTable(al.constraintTable), agent(*al.agents[0])
{
    if(al.agents.size()!=1)
        cout<<"Single Planning can only have 1 agent in al->agents"<<endl;

    this->screen = screen;
    if (agent.position < 0)
	    this->start_location = agent.initial_location;
	else
        this->start_location = agent.position;
	this->goal_location = agent.goal_location;

	this->map_size = ml.cols*ml.rows;

	this->LL_num_expanded = 0;
	this->LL_num_generated = 0;

	this->num_col = ml.cols;

}


bool SIPP::search() // TODO: weighted SIPP
{

    assert(f_w < 1.001);
    Time::time_point start_clock = Time::now();

    LL_num_expanded = 0;
    LL_num_generated = 0;

    hashtable_t::iterator it;  // will be used for find()


    // generate start and add it to the OPEN list
    LL_num_generated++;
    int h = 1 + my_heuristic[start_location].get_hval(agent.heading);
    SIPPNode*  start = new SIPPNode(-1,
            agent.malfunction_left, // malfunction_left is the earliest timestep when the agent can start to move
            h, nullptr,
            agent.malfunction_left, // malfunction_left is the earliest timestep when the agent can start to move
            make_pair(0,constraintTable.length_max + 1));


    if (agent.status != 0) {
        start->loc = start_location;
        start->h_val--;

        if (constraintTable.get_latest_constrained_timestep(start_location) > 0)
        {
            int t_max = agent.malfunction_left + 1;
            while(!constraintTable.is_constrained(agent.agent_id, start_location, t_max, -1) &&
                t_max <= constraintTable.length_max)
                t_max++;
            start->interval = make_pair(0, t_max);
        }
    }


    start->heading = agent.heading;
    start->time_generated = 0;
    start->malfunction_left = agent.malfunction_left;
    start->position_fraction = agent.position_fraction;
    start->exit_heading = agent.exit_heading;
    if (start->position_fraction>=1) {
        list<Transition> temp;
        ml.get_transitions(temp, start_location, start->heading, true);
        if (temp.size() == 1) { //if not on a cross, exit_heading is not accurate as flatland env only provide an move forward action.
            start->exit_loc = temp.front().location;
            start->exit_heading = temp.front().heading;
        }
        else //if on a cross, exit_heading is always accurate, as move forwad must be along agent's current heading.
            start->exit_loc = start_location + ml.moves_offset[start->exit_heading];
        int t_max = agent.malfunction_left + 1;
        while(constraintTable.is_constrained(agent.agent_id, start->exit_loc, t_max, start_location) && t_max <= constraintTable.length_max)
            t_max++;
        start->interval = make_pair(0, t_max);
    }
    start->open_handle = open_list.push(start);
    start->in_openlist = true;
    allNodes_table.insert(start);


    int time_generated = 0;
    int time_check_count = 0;
    fsec runtime;

    while (!open_list.empty())
    {
        if (LL_num_generated / 1000 > time_check_count && time_limit != 0) {
            runtime = Time::now() - start_clock;
            time_check_count = LL_num_generated / 1000;
            if (runtime.count() > time_limit) {
                releaseClosedListNodes();
                return false;
            }
        }

        auto curr = open_list.top(); open_list.pop();
        curr->in_openlist = false;
        LL_num_expanded++;

        // check if the popped node is a goal
        if (curr->loc == goal_location)
        {
            updatePath(curr);
            releaseClosedListNodes();
            return true;
        }


        list<Transition> transitions;
        if(curr->loc == -1){
            transitions.emplace_back(start_location, curr->heading,
                                     curr->position_fraction, curr->exit_loc, curr->exit_heading);
        }
        else if ( curr->position_fraction>=1){

            transitions.emplace_back(curr->exit_loc, curr->exit_heading);

        }
        else
            ml.get_transitions(transitions, curr->loc, curr->heading, true);


        for (const auto& move : transitions)
        {
            int next_id = move.location;
            int next_heading = move.heading;
            int next_h_val = my_heuristic[next_id].get_hval(next_heading);
            list<Interval> intervals;
            getSafeIntervals(curr->loc, curr->timestep, curr->interval, next_id, next_h_val, intervals);

            for (auto& next_interval : intervals)
            {
                time_generated += 1;
                int next_timestep = max(curr->timestep + 1, next_interval.first);

                // generate (maybe temporary) node
                auto next = new SIPPNode(next_id, next_timestep, next_h_val, curr, next_timestep, next_interval);
                next->heading = next_heading;
                next->time_generated = time_generated;
                next->position_fraction = move.position_fraction;
                next->exit_heading = move.exit_heading;
                next->exit_loc = move.exit_loc;
                if (curr->loc == -1)
                    next->show_time = next_timestep;
                else
                    next->show_time = curr->show_time;
                // try to retrieve it from the hash table
                it = allNodes_table.find(next);
                if (it == allNodes_table.end())
                {
                    next->open_handle = open_list.push(next);
                    next->in_openlist = true;
                    LL_num_generated++;
                    allNodes_table.insert(next);
                }
                else
                {  // update existing node's if needed (only in the open_list)
                    auto existing_next = (*it);
                    if (existing_next->g_val > next->g_val ||
                        (existing_next->g_val == next->g_val && existing_next->show_time < next->show_time))
                    {// update existing node
                        existing_next->g_val = next->g_val;
                        existing_next->timestep = next->timestep;
                        existing_next->interval = next->interval;
                        existing_next->show_time = next->show_time;
                        existing_next->parent = curr;
                        if (existing_next->in_openlist)// if its in the open list
                        {
                            open_list.increase(existing_next->open_handle);  // increase because f-val improved
                        }
                        else // if its in the closed list (reopen)
                        {
                            existing_next->open_handle = open_list.push(existing_next);
                            existing_next->in_openlist = true;
                        }
                    }
                    delete(next);  // not needed anymore -- we already generated it before
                }  // end update an existing node
            }
        }  // end for loop that generates successors
    }  // end while loop

    // no path found
    releaseClosedListNodes();
    return false;
}


void SIPP::getSafeIntervals(int prev_loc, int prev_timestep,
        const Interval& prev_interval, int next_loc, int next_h, list<Interval>& intervals)
{
    int t_min = prev_timestep + 1;
    while (t_min <= prev_interval.second)
    {
        // find the earliest timestep when we can move to next loc
        while(constraintTable.is_constrained(agent.agent_id, next_loc, t_min, prev_loc))
        {
            if (t_min == prev_interval.second)
            {
                return;
            }
            t_min++;
        }
        if (constraintTable.get_latest_constrained_timestep(next_loc) <= t_min) // no constraints after this timestep
        {
            if (t_min < constraintTable.length_max + 1)
            {
                intervals.emplace_back(t_min, constraintTable.length_max + 1);
            }
            return;
        }
        // find the latest timestep when we can move to next loc
        int t_max = t_min;
        while(!constraintTable.blocked(next_loc, t_max)) // no vertex conflict
            t_max++;
        if (t_min < t_max)
        {
            intervals.emplace_back(t_min, t_max);
            /*int t = prev_timestep + 1;
            while (t < t_min - 1)
            {
                assert(!constraintTable.blocked(prev_loc, t));
                t++;
            }
            assert(!constraintTable.is_constrained(agent.agent_id, next_loc, t_min, prev_loc));
            while (t < t_max - 1)
            {
                assert(!constraintTable.blocked(prev_loc, t));
                t++;
            }
            assert(!constraintTable.is_constrained(agent.agent_id, next_loc, t_max - 1, prev_loc));*/
        }
        t_min = t_max + 1;
    }
}

void SIPP::updatePath(SIPPNode* goal)
{
    path.resize(goal->timestep + 1);
    const auto* curr = goal;

    if (agent.status > 0) {//go as early as possible
        for (int t = goal->timestep; t >=0; t--)
        {
            if (t < curr->interval.first && curr->parent!= nullptr)
            {
                curr = curr->parent;
            }
            assert((curr->parent== nullptr || t>curr->parent->timestep) && t>=curr->interval.first && t<=curr->interval.second);

            path[t].location = curr->loc;
            path[t].heading = curr->heading;
            path[t].position_fraction = curr->position_fraction;
            path[t].exit_heading = curr->exit_heading;
            path[t].exit_loc = curr->exit_loc;

        }

    }
    else{//go as late as possible
        int t_start = goal->timestep;
        std::stack<const SIPPNode*> states;
        while (curr->parent != nullptr) // non-root node
        {
            states.push(curr);
            t_start = min(t_start - 1, curr->parent->interval.second - 1); // latest timestep to leave curr->parent->loc
            curr = curr->parent;
        }
        const auto* prev = states.top(); states.pop();
        curr = states.top(); states.pop();
        for (int t = t_start + 1; t < goal->timestep; t++)
        {
            if (t > t_start + 1 && t >= curr->interval.first)
            {
                prev = curr;
                curr = states.top(); states.pop();
            }
            assert(prev->interval.first <= t && t < prev->interval.second);
            path[t].location = prev->loc;
            path[t].heading = prev->heading;
            path[t].position_fraction = prev->position_fraction;
            path[t].exit_heading = prev->exit_heading;
            path[t].exit_loc = prev->exit_loc;
            assert(!constraintTable.is_constrained(agent.agent_id, path[t].location, t, path[t - 1].location));
        }

        assert(states.empty());
        assert(!constraintTable.is_constrained(agent.agent_id, path[goal->timestep].location,
                                               goal->timestep, path[goal->timestep - 1].location));
        path[goal->timestep].location = goal->loc;
        path[goal->timestep].heading = goal->heading;
        path[goal->timestep].position_fraction = goal->position_fraction;
        path[goal->timestep].exit_heading = goal->exit_heading;
        path[goal->timestep].exit_loc = goal->exit_loc;
    }
    if (agent.malfunction_left > 0)
    {
        for (int t = 0; t < min(agent.malfunction_left, (int)path.size()); t++)
            path[t].malfunction_left = agent.malfunction_left - t;
    }

}
