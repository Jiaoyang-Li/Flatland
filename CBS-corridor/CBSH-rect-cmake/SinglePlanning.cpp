#include "SinglePlanning.h"

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
	    start = new LLNode(-1, 0, (1 + my_heuristic[start_location].get_hval(agent.heading)), nullptr, 0, false);
    else
        start = new LLNode(start_location, 0, my_heuristic[start_location].get_hval(agent.heading), nullptr, 0,  false);
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
                releaseClosedListNodes(&allNodes_table);

                open_list.clear();

                allNodes_table.clear();
				return false;
			}
		}

		LLNode* curr = open_list.top(); open_list.pop();
		curr->in_openlist = false;

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

			allNodes_table.clear();
			return true;
			//}
		}
		

		list<Transition> transitions;
		if(flat){
            if(curr->loc == -1){
                Transition move2;
                move2.location = start_location;
                move2.heading = curr->heading;
                move2.position_fraction = curr->position_fraction;
                move2.exit_loc = curr->exit_loc;
                move2.exit_heading = curr->exit_heading;
                transitions.push_back(move2);
            }
            else
                ml.get_transitions(transitions, curr->loc, curr->heading, true);
		}
		else if (curr->malfunction_left>0 ){
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
		else if ( curr->position_fraction>=1 && curr->exit_heading>=0 ){

                if (constraintTable.is_constrained(agent.agent_id, curr->exit_loc, curr->timestep+1,curr->loc)) {
                    Transition move2;
                    move2.location = curr->loc;
                    move2.heading = curr->heading;
                    move2.position_fraction = curr->position_fraction;
                    move2.exit_loc = curr->exit_loc;
                    move2.exit_heading = curr->exit_heading;
                    transitions.push_back(move2);
                }
                else{
                    Transition move;
                    move.location = curr->exit_loc;
                    move.heading = curr->exit_heading;
                    move.position_fraction = 0;
                    transitions.push_back(move);
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
                    if (next_h_val < MAX_COST){
                        next_h_val = next_h_val*f_w;
                    }
                }
                else
                    next_h_val = curr->h_val;
                assert(next_h_val >= 0 && next_h_val<=MAX_COST);


				if (next_h_val>= MAX_COST)
					continue;

//                cout<<"Next: "<<next_h_val<<","<< next_id/num_col <<", "<<next_id%num_col<<", heading: "<<next_heading<< endl;
//                for (auto heading : my_heuristic[next_id].heading){
//                    cout << heading<< ",";
//                }
//                cout <<endl;


                // generate (maybe temporary) node
				auto next = new LLNode(next_id, next_g_val, next_h_val,	curr, next_timestep, false);
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
                    next->open_handle = open_list.push(next);


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

                            existing_next->parent = curr;
							existing_next->open_handle = open_list.push(existing_next);
							existing_next->in_openlist = true;

						}
					}  // end update a node in closed list
				}  // end update an existing node
			}// end if case forthe move is legal
		}  // end for loop that generates successors
		//cout << "focal list size"<<focal_list.size() << endl;
		// update FOCAL if min f-val increased
		if (open_list.empty())  // in case OPEN is empty, no path found
			break;


	}  // end while loop

	  // no path found
	releaseClosedListNodes(&allNodes_table);
	open_list.clear();
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
    ml(ml),al(al), my_heuristic(*al.agents[0]->heuristics),constraintTable(al.constraintTable), agent(*al.agents[0])
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

	this->num_col = ml.cols;

}
