#include "SinglePlanning.h"

#include <iostream>


void SinglePlanning::updatePath(LLNode* goal)
{
	path.resize(goal->timestep + 1);
	LLNode* curr = goal;
	num_of_conf = goal->num_internal_conf;
	for(int t = goal->timestep; t >= 0; t--)
	{

		path[t].location = curr->loc;
		path[t].actionToHere = curr->heading;
		path[t].heading = curr->heading;
		path[t].position_fraction = curr->position_fraction;
		path[t].malfunction_left = curr->malfunction_left;
		path[t].exit_heading = curr->exit_heading;
		path[t].exit_loc = curr->exit_loc;

		path[t].conflist = nullptr;
        if (t == goal->timestep && curr->loc != goal_location) {
			path[t].malfunction = true;
		}

		curr = curr->parent;
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// return true if a path found (and updates vector<int> path) or false if no path exists

bool SinglePlanning::search()
{
    Time::time_point start_clock = Time::now();

    LL_num_expanded = 0;
    LL_num_generated = 0;

	hashtable_t::iterator it;  // will be used for find()


	 // generate start and add it to the OPEN list
	LLNode* start;
	if (agent.status == 0)
	    start = new LLNode(-1, 0, 1 + my_heuristic[start_location].get_hval(agent.heading)/agent.speed, nullptr, 0, 0, false); // TODO::Shouldn't the h value be divided by its speed? Yes it should
    else
        start = new LLNode(start_location, 0, my_heuristic[start_location].get_hval(agent.heading)/agent.speed, nullptr, 0, 0, false);

    start->heading = agent.heading;
    LL_num_generated++;
	start->open_handle = open_list.push(start);
	start->focal_handle = focal_list.push(start);
	start->in_openlist = true;
	start->time_generated = 0;
	start->malfunction_left = agent.malfunction_left;

    
	start->position_fraction = agent.position_fraction;
	start->exit_heading = agent.exit_heading;
	if (start->exit_heading >= 0) {
	    //I forget why I have this here, it must have a reason.
	    //Exit_loc and exit_heading only have influence on trains with speed!=1.
		list<Transition> temp;
		ml.get_transitions(temp, start_location, start->heading, true);
		if (temp.size() == 1) {
			start->exit_loc = temp.front().location;
			start->exit_heading = temp.front().heading;
		}
		else
			start->exit_loc = start_location + ml.moves_offset[start->exit_heading];
	}

	int start_h_val = my_heuristic[start_location].get_hval(agent.heading) / agent.speed;
	if (start->exit_loc >= 0 && agent.speed < 1) {
		int h1 = my_heuristic[start_location].get_hval(agent.heading)/ agent.speed;
		int h2 = my_heuristic[start->exit_loc].get_hval(start->exit_heading)/ agent.speed;
		start_h_val = h1 + (h2 - h1)*agent.position_fraction;

	}
	start->h_val = start_h_val;


	allNodes_table.insert(start);
	min_f_val = start->getFVal();

    focal_threshold = f_w * min_f_val;

	int time_generated = 0;
	int time_check_count = 0;
    fsec runtime;
	/*for (int h = 0; h < my_heuristic.size();h++) {
		for (int heading = 0; heading<5;heading++)
			std::cout << "(" << h << ": heading:"<<-1 <<": "<< my_heuristic[h].heading[-1] << ")";
	}*/

	while (!focal_list.empty()) 
	{
		if (LL_num_generated / 10000 > time_check_count && time_limit != 0) {
			runtime = Time::now() - start_clock;
			time_check_count = LL_num_generated / 10000;
			if (runtime.count() > time_limit) {
                releaseClosedListNodes(&allNodes_table);

                open_list.clear();
                focal_list.clear();

                allNodes_table.clear();
				return false;
			}
		}

		LLNode* curr = focal_list.top(); focal_list.pop();
		open_list.erase(curr->open_handle);

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

			releaseClosedListNodes(&allNodes_table);

			open_list.clear();
			focal_list.clear();

			allNodes_table.clear();
			return true;
			//}
		}
		

		list<Transition> transitions;
		if (curr->malfunction_left>0){
            Transition move;
            move.location = curr->loc;
            move.heading = curr->heading;
            move.position_fraction = curr->position_fraction;
            move.exit_loc = curr->exit_loc;
            move.exit_heading = curr->exit_heading;
            transitions.push_back(move);
		}
		else if(curr->loc == -1){
            Transition move;
			move.location = -1;
			move.heading = curr->heading;
			move.position_fraction = curr->position_fraction;
			move.exit_loc = curr->exit_loc;
			move.exit_heading = curr->exit_heading;
			transitions.push_back(move);
            
            Transition move2;
			move2.location = start_location;
			move2.heading = curr->heading;
			move2.position_fraction = curr->position_fraction;
			move2.exit_loc = curr->exit_loc;
			move2.exit_heading = curr->exit_heading;
			transitions.push_back(move2);
            
        }
		else if (curr->position_fraction +agent.speed >= 0.97) {
			if (curr->position_fraction == 0)
			    ml.get_transitions(transitions, curr->loc, curr->heading, false);
			else {
				Transition move;
				move.location = curr->exit_loc;
				move.heading = curr->exit_heading;
				move.position_fraction = 0;
				transitions.push_back(move);
			}
		}
		else if (curr->position_fraction == 0) {
		    ml.get_exits(transitions, curr->loc, curr->heading, agent.speed, false);

		}
		else { //<0.97 and po_frac not 0
			

			Transition move2;
			move2.location = curr->loc;
			move2.heading = curr->heading;
			move2.position_fraction = curr->position_fraction + agent.speed;
			move2.exit_loc = curr->exit_loc;
			move2.exit_heading = curr->exit_heading;
			transitions.push_back(move2);


		}


		for (const auto& move : transitions)
		{
			int next_id = move.location;
			time_generated += 1;
			int next_timestep = curr->timestep + 1;


			if (!constraintTable.is_constrained(agent.agent_id, next_id, next_timestep)) //&&
				//!constraintTable.is_constrained(curr->loc * map_size + next_id, next_timestep)) // TODO:: for k-robust cases, we do not need to check edge constraint?
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

                if (next_id!=-1)
                    next_h_val = my_heuristic[next_id].get_hval(next_heading)/agent.speed;
                else
                    next_h_val = curr->h_val;

				if (next_id!=-1 && move.exit_loc >= 0 && agent.speed<1) {
					int h1 = my_heuristic[next_id].get_hval(next_heading)/agent.speed;
					int h2 = my_heuristic[move.exit_loc].get_hval(move.exit_heading)/agent.speed;
					next_h_val = h1 + (h2-h1)*(move.position_fraction);

				}
				if (next_g_val + next_h_val> constraintTable.length_max)
					continue;

//                cout<<"Next: "<<next_h_val<<","<< next_id/num_col <<", "<<next_id%num_col<<", heading: "<<next_heading<< endl;
//                for (auto heading : my_heuristic[next_id].heading){
//                    cout << heading<< ",";
//                }
//                cout <<endl;


                // generate (maybe temporary) node
				auto next = new LLNode(next_id, next_g_val, next_h_val,	curr, next_timestep, 0, false);
				next->heading = next_heading;
				next->actionToHere = move.heading;
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
					if (next->getFVal() <= focal_threshold)
					{
						next->focal_handle = focal_list.push(next);
					}

					allNodes_table.insert(next);
				}
				else
				{  // update existing node's if needed (only in the open_list)
					delete(next);  // not needed anymore -- we already generated it before
					LLNode* existing_next = (*it);

					if (existing_next->in_openlist)
					{  // if its in the open list
						if (existing_next->getFVal() > next_g_val + next_h_val)
						{
							// if f-val decreased through this new path (or it remains the same and there's less internal conflicts)
							bool add_to_focal = false;  // check if it was above the focal bound before and now below (thus need to be inserted)
							bool update_in_focal = false;  // check if it was inside the focal and needs to be updated (because f-val changed)
							bool update_open = false;
							if ((next_g_val + next_h_val) <= focal_threshold)
							{  // if the new f-val qualify to be in FOCAL
								if (existing_next->getFVal() > focal_threshold)
									add_to_focal = true;  // and the previous f-val did not qualify to be in FOCAL then add
								else
									update_in_focal = true;  // and the previous f-val did qualify to be in FOCAL then update
							}
							if (existing_next->getFVal() > next_g_val + next_h_val)
								update_open = true;
							// update existing node
							existing_next->g_val = next_g_val;
							existing_next->h_val = next_h_val;
							existing_next->parent = curr;
							if (update_open) 
								open_list.increase(existing_next->open_handle);  // increase because f-val improved
							if (add_to_focal) 
								existing_next->focal_handle = focal_list.push(existing_next);
							if (update_in_focal) 
								focal_list.update(existing_next->focal_handle);  // should we do update? yes, because number of conflicts may go up or down
						}				
					}
					else 
					{  // if its in the closed list (reopen)
						if (existing_next->getFVal() > next_g_val + next_h_val )
						{
							// if f-val decreased through this new path (or it remains the same and there's less internal conflicts)
							existing_next->g_val = next_g_val;
							existing_next->h_val = next_h_val;
							existing_next->parent = curr;
							existing_next->open_handle = open_list.push(existing_next);
							existing_next->in_openlist = true;
							if (existing_next->getFVal() <= focal_threshold)
                                existing_next->focal_handle = focal_list.push(existing_next);
						}
					}  // end update a node in closed list
				}  // end update an existing node
			}// end if case forthe move is legal
		}  // end for loop that generates successors
		//cout << "focal list size"<<focal_list.size() << endl;
		// update FOCAL if min f-val increased
		if (open_list.empty())  // in case OPEN is empty, no path found
			break;
		LLNode* open_head = open_list.top();

        //assert(open_head->getFVal() >= min_f_val);
		if (open_head->getFVal() > min_f_val) 
		{
			min_f_val = open_head->getFVal();
			double new_focal_threshold = std::max(f_w * min_f_val, min_f_val);
            if (new_focal_threshold > focal_threshold)
            {
                for (LLNode* n : open_list)
                {

                    if (n->getFVal() > focal_threshold && n->getFVal() <= new_focal_threshold)
                    {
                        n->focal_handle = focal_list.push(n);
                    }
                }
                focal_threshold = new_focal_threshold;
            }
		}


	}  // end while loop

	assert(min_f_val >= constraintTable.length_max);
	  // no path found
	releaseClosedListNodes(&allNodes_table);
	open_list.clear();
	focal_list.clear();
	allNodes_table.clear();
	return false;
}


inline void SinglePlanning::releaseClosedListNodes(hashtable_t* allNodes_table)
{

	hashtable_t::iterator it;
	for (it = allNodes_table->begin(); it != allNodes_table->end(); ++it) {

			delete (*it);
	}
}


SinglePlanning::SinglePlanning(const FlatlandLoader& ml, AgentsLoader& al, double f_w, float time_limit, options option):
    ml(ml),al(al), my_heuristic(al.agents[0]->heuristics),constraintTable(al.constraintTable), agent(*al.agents[0])
{
    if(al.agents.size()!=1)
        cout<<"Single Planning can only have 1 agent in al->agents"<<endl;

    this->screen = screen;
    this->time_limit= time_limit;
	this->f_w = f_w;
	this->start_location = ml.linearize_coordinate(agent.position.first, agent.position.second);
	this->goal_location = ml.linearize_coordinate(agent.goal_location.first, agent.goal_location.second);

	this->map_size = ml.cols*ml.rows;

	this->LL_num_expanded = 0;
	this->LL_num_generated = 0;

	this->focal_threshold = 0;
	this->min_f_val = 0;

	this->num_col = ml.cols;

}
