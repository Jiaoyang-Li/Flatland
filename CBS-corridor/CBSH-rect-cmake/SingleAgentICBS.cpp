#include "SingleAgentICBS.h"

#include <iostream>
#include <ctime>

template<class Map>
void SingleAgentICBS<Map>::updatePath(LLNode* goal, std::vector<PathEntry> &path,ReservationTable* res_table)
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
		path[t].next_malfunction = curr->next_malfunction;
		path[t].exit_heading = curr->exit_heading;
		path[t].exit_loc = curr->exit_loc;
		delete path[t].conflist;
		if (t!=0)
        {
            path[t].conflist =  res_table->findConflict(agent_id, curr->parent->loc, curr->loc, t-1, kRobust);
        }
        else
        {
            path[t].conflist = nullptr;
        }
        if (t == goal->timestep && curr->loc != goal_location) {
			path[t].malfunction = true;
		}

		curr->conflist = nullptr;
		curr = curr->parent;
	}
}


// iterate over the constraints ( cons[t] is a list of all constraints for timestep t) and return the latest
// timestep which has a constraint involving the goal location
/*template<class Map>
int SingleAgentICBS<Map>::extractLastGoalTimestep(int goal_location, const std::vector< std::list< std::pair<int, int> > >* cons) {
	if (cons != nullptr) {
		for (int t = static_cast<int>(cons->size()) - 1; t > 0; t--) 
		{
			for (const auto it : cons->at(t).begin(); it != cons->at(t).end(); ++it)
			{
				if (std::get<0>(*it) == goal_location && it->second < 0) 
				{
					return (t);
				}
			}
		}
	}
	return -1;
}*/



template<class Map>
int SingleAgentICBS<Map>::numOfConflictsForStep(int curr_id, int next_id, int next_timestep, const bool* res_table, int max_plan_len) {
	int retVal = 0;
	if (next_timestep >= max_plan_len) {
		// check vertex constraints (being at an agent's goal when he stays there because he is done planning)
		if (res_table[next_id + (max_plan_len - 1)*map_size] == true)
			retVal++;
		// Note -- there cannot be edge conflicts when other agents are done moving
	}
	else {
		// check vertex constraints (being in next_id at next_timestep is disallowed)
		if (res_table[next_id + next_timestep*map_size] == true)
			retVal++;
		// check edge constraints (the move from curr_id to next_id at next_timestep-1 is disallowed)
		// which means that res_table is occupied with another agent for [curr_id,next_timestep] and [next_id,next_timestep-1]
		if (res_table[curr_id + next_timestep*map_size] && res_table[next_id + (next_timestep - 1)*map_size])
			retVal++;
	}
	return retVal;
}

template<class Map>
bool SingleAgentICBS<Map>::validMove(int curr, int next) const
{
	if (next < 0 || next >= map_size)
		return false;
	int curr_x = curr / num_col;
	int curr_y = curr % num_col;
	int next_x = next / num_col;
	int next_y = next % num_col;
	return abs(next_x - curr_x) + abs(next_y - curr_y) < 2;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// return true if a path found (and updates vector<int> path) or false if no path exists
// focal_makespan is the makespan threshold of the focal list used in the high-level.
// When the path length is shorter than focal_makespan, the low-level performs a focal search;
// When the path length exceeds focal_makespan, it performs an A* search.
template<class Map>
bool SingleAgentICBS<Map>::findPath(std::vector<PathEntry> &path, double f_weight, int focal_makespan,
        ConstraintTable& constraint_table,
	ReservationTable* res_table, size_t max_plan_len, double lowerbound, Time::time_point start_clock ,int time_limit)
{

	num_expanded = 0;
	num_generated = 0;

	hashtable_t::iterator it;  // will be used for find()


	 // generate start and add it to the OPEN list
	auto start = new LLNode(-1, 0, my_heuristic[start_location].get_hval(start_heading), nullptr, 0, 0, false); // TODO::Shouldn't the h value be divided by its speed?
	start->heading = start_heading;
	num_generated++;
	start->open_handle = open_list.push(start);
	start->focal_handle = focal_list.push(start);
	start->in_openlist = true;
	start->time_generated = 0;
	OldConfList* conflicts = res_table->findConflict(agent_id, start->loc, start->loc, -1, kRobust);
	start->conflist = conflicts;
	start->num_internal_conf= conflicts->size();

    
	start->position_fraction = al->agents[agent_id]->position_fraction;
	start->exit_heading = al->agents[agent_id]->exit_heading;
	if (start->exit_heading >= 0) {
		list<Transition> temp;
		ml->get_transitions(temp, start_location, start->heading, true);
		if (temp.size() == 1) {
			start->exit_loc = temp.front().location;
			start->exit_heading = temp.front().heading;
		}
		else
			start->exit_loc = start_location + ml->moves_offset[start->exit_heading];
	}

	int start_h_val = my_heuristic[start_location].get_hval(start_heading) / al->agents[agent_id]->speed;
	if (start->exit_loc >= 0 && al->agents[agent_id]->speed < 1) {
		int h1 = my_heuristic[start_location].get_hval(start_heading);
		int h2 = my_heuristic[start->exit_loc].get_hval(start->exit_heading);
		start_h_val = h1 / al->agents[agent_id]->speed
			- (h2 - h1)*al->agents[agent_id]->position_fraction;

	}
	start->h_val = start_h_val;


	allNodes_table.insert(start);
	min_f_val = start->getFVal();

    focal_threshold = std::max(lowerbound, std::min(f_weight * min_f_val, (double)focal_makespan));

	int time_generated = 0;
	int time_check_count = 0;
    fsec runtime;
	/*for (int h = 0; h < my_heuristic.size();h++) {
		for (int heading = 0; heading<5;heading++)
			std::cout << "(" << h << ": heading:"<<-1 <<": "<< my_heuristic[h].heading[-1] << ")";
	}*/

	while (!focal_list.empty()) 
	{
		if (num_generated / 10000 > time_check_count && time_limit != 0) {
			runtime = Time::now() - start_clock;
			time_check_count = num_generated / 10000;
			if (runtime.count() > time_limit) {
				return false;
			}
		}

		LLNode* curr = focal_list.top(); focal_list.pop();
		open_list.erase(curr->open_handle);

// 		assert(curr->h_val >=0);

		curr->in_openlist = false;
		num_expanded++;
		//cout << "focal size " << focal_list.size() << endl;
		//cout << "goal_location: " << goal_location << " curr time: " << curr->timestep << " length_min: " << constraint_table.length_min << endl;
		// check if the popped node is a goal
		if ((curr->loc == goal_location ) /*|| (curr->parent!= nullptr && curr->next_malfunction==0 && curr->parent->next_malfunction ==1)*/)
		{

			updatePath(curr, path, res_table);

			releaseClosedListNodes(&allNodes_table);

			open_list.clear();
			focal_list.clear();

			allNodes_table.clear();
			return true;
			//}
		}
		

		list<Transition> transitions;
		if(curr->loc == -1){
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
		else if (curr->position_fraction + al->agents[agent_id]->speed >= 0.97) {
			if (curr->position_fraction == 0)
			    ml->get_transitions(transitions, curr->loc, curr->heading, false);
			else {
				Transition move;
				move.location = curr->exit_loc;
				move.heading = curr->exit_heading;
				move.position_fraction = 0;
				transitions.push_back(move);
			}
		}
		else if (curr->position_fraction == 0) {
		    ml->get_exits(transitions, curr->loc, curr->heading, al->agents[agent_id]->speed, false);

		}
		else { //<0.97 and po_frac not 0
			

			Transition move2;
			move2.location = curr->loc;
			move2.heading = curr->heading;
			move2.position_fraction = curr->position_fraction + al->agents[agent_id]->speed;
			move2.exit_loc = curr->exit_loc;
			move2.exit_heading = curr->exit_heading;
			transitions.push_back(move2);


		}


		for (const auto& move : transitions)
		{
			int next_id = move.location;
			time_generated += 1;
			int next_timestep = curr->timestep + 1;


			if (!constraint_table.is_constrained(next_id, next_timestep)) //&&
				//!constraint_table.is_constrained(curr->loc * map_size + next_id, next_timestep)) // TODO:: for k-robust cases, we do not need to check edge constraint?
			{

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
                    next_h_val = my_heuristic[next_id].get_hval(next_heading);
                else
                    next_h_val = curr->h_val;

				if (next_id!=-1 && move.exit_loc >= 0 && al->agents[agent_id]->speed<1) {
					int h1 = my_heuristic[next_id].get_hval(next_heading);
					int h2 = my_heuristic[move.exit_loc].get_hval(move.exit_heading);
					next_h_val = h1
						+ (h2-h1)*(move.position_fraction/1);
                    next_h_val = round( next_h_val * 10000.0 ) / 10000.0;

				}
				if (next_g_val + next_h_val*al->agents[agent_id]->speed > constraint_table.length_max)
					continue;


				int next_internal_conflicts = res_table->countConflict(agent_id, curr->loc, next_id, curr->timestep, kRobust);



                // generate (maybe temporary) node
				auto next = new LLNode(next_id, next_g_val, next_h_val,	curr, next_timestep, next_internal_conflicts, false);
				next->heading = next_heading;
				next->actionToHere = move.heading;
				next->time_generated = time_generated;
				next->position_fraction = next_position_fraction;
				next->exit_heading = move.exit_heading;
				next->exit_loc = move.exit_loc;
				next->show_time = next_show_time;

				// try to retrieve it from the hash table
				it = allNodes_table.find(next);
				if (it == allNodes_table.end())
				{

					next->open_handle = open_list.push(next);
					next->in_openlist = true;
					num_generated++;
					if (next->getFVal() <= focal_threshold)
					{
						next->focal_handle = focal_list.push(next);
					}

					allNodes_table.insert(next);
					next->conflist = conflicts; // TODO: Can I delete this line?
				}
				else
				{  // update existing node's if needed (only in the open_list)
					delete(next);  // not needed anymore -- we already generated it before
					LLNode* existing_next = (*it);

					if (existing_next->in_openlist)
					{  // if its in the open list
						if (existing_next->getFVal() > next_g_val + next_h_val ||
							(existing_next->getFVal() == next_g_val + next_h_val && existing_next->num_internal_conf > next_internal_conflicts))
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
							existing_next->num_internal_conf = next_internal_conflicts;
							delete(existing_next->conflist);
							existing_next->conflist = conflicts;
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
						if (existing_next->getFVal() > next_g_val + next_h_val ||
							(existing_next->getFVal() == next_g_val + next_h_val && existing_next->num_internal_conf > next_internal_conflicts)) 
						{
							// if f-val decreased through this new path (or it remains the same and there's less internal conflicts)
							existing_next->g_val = next_g_val;
							existing_next->h_val = next_h_val;
							existing_next->parent = curr;
							existing_next->num_internal_conf = next_internal_conflicts;
							existing_next->open_handle = open_list.push(existing_next);
							existing_next->in_openlist = true;
							delete(existing_next->conflist);
							existing_next->conflist = conflicts;
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

        assert(open_head->getFVal() >= min_f_val);
		if (open_head->getFVal() > min_f_val) 
		{
			min_f_val = open_head->getFVal();
			double new_focal_threshold = std::max(lowerbound, std::max(std::min(f_weight * min_f_val, (double)focal_makespan), min_f_val));
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

	assert(min_f_val >= constraint_table.length_max);
	  // no path found
	releaseClosedListNodes(&allNodes_table);
	open_list.clear();
	focal_list.clear();
	allNodes_table.clear();
	return false;
}

template<class Map>
inline void SingleAgentICBS<Map>::releaseClosedListNodes(hashtable_t* allNodes_table)
{

	hashtable_t::iterator it;
	for (it = allNodes_table->begin(); it != allNodes_table->end(); ++it) {

			delete (*it);
	}
}

template<class Map>
SingleAgentICBS<Map>::SingleAgentICBS(int start_location, int goal_location,  const Map* ml1, AgentsLoader* al,int agent_id, int start_heading, int kRobust):
    ml(ml1), my_heuristic(al->agents[agent_id]->heuristics)
{
	this->al = al;
	this->agent_id = agent_id;
	this->start_heading = start_heading;

	this->start_location = start_location;
	this->goal_location = goal_location;
	

	this->map_size = ml->cols*ml->rows;

	this->num_expanded = 0;
	this->num_generated = 0;

	this->focal_threshold = 0;
	this->min_f_val = 0;

	this->num_col = ml->cols;

	this->kRobust = kRobust;
}

template<class Map>
SingleAgentICBS<Map>::~SingleAgentICBS()
= default;

template class SingleAgentICBS<MapLoader>;
template class SingleAgentICBS<FlatlandLoader>;
