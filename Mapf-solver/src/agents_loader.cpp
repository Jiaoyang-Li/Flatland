/*
 * @author: Team An_old_driver
 * @created: 09-2020
 * Copyright (c) 2020 The University of Southern California. All Rights Reserved.
 * Copyrights licensed under an Academic/non-profit use license.
 * See the accompanying LICENSE file for terms.
*/

#include "agents_loader.h"
#include <string>
#include <cstring>
#include <iostream>
#include <cassert>
#include <fstream>
#include<boost/tokenizer.hpp>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <utility>
#include <algorithm>  // for remove_if
#include <ctime>
using namespace boost;
using namespace std;
namespace p = boost::python;


int RANDOM_WALK_STEPS = 100000;

std::ostream& operator<<(std::ostream& os, const Agent& agent) {
	os << "Initial: " << agent.initial_location << ","
       << "Goal_location: " << agent.goal_location << ","
       << "Status: " << agent.status << ", Heading: " << agent.heading
		<< ", malfunction_left: " << agent.malfunction_left
		<< ", next_malfuntion: " << agent.next_malfunction
		<< ", speed: " << agent.speed
		<< ", position_fraction: " << agent.position_fraction
        ;
    return os;
}

AgentsLoader::AgentsLoader(const FlatlandLoader &ml, p::object agents) {
	int agentsNum = p::len(agents);
	this->num_of_agents_all = agentsNum;
	///this->heuristics.resize(num_of_agents_all);
    this->agents_all.resize(num_of_agents_all);
    this->paths_all.resize(num_of_agents_all);
    this->constraintTable.num_of_agents = num_of_agents_all;
    //this->initial_locations_all.resize(num_of_agents_all);
    //this->goal_locations_all.resize(num_of_agents_all);
    //this->headings_all.resize(num_of_agents_all);
	for (int i = 0; i < num_of_agents_all; i++) {
		pair<int, int> initial;
		pair<int, int> goal;
		int status = p::extract<int>(agents[i].attr("status"));
        p::tuple iniTuple(agents[i].attr("initial_position"));
        initial.first = p::extract<int>(p::long_(iniTuple[0]));
        initial.second = p::extract<int>(p::long_(iniTuple[1]));
		
		
		p::tuple goalTuple(agents[i].attr("target"));
		goal.first = p::extract<int>(p::long_(goalTuple[0]));
		goal.second = p::extract<int>(p::long_(goalTuple[1]));
		int heading = p::extract<int>(p::long_(agents[i].attr("initial_direction")));
		//this->initial_locations_all[i] = initial;
		//this->goal_locations_all[i] = goal;
		//this->headings_all[i] = heading;

		int malfunction = p::extract<int>(agents[i].attr("malfunction_data")["malfunction"]);
		int next_malfunction = p::extract<int>(p::long_(agents[i].attr("malfunction_data")["next_malfunction"]));
		float malfunction_rate = p::extract<int>(p::long_(agents[i].attr("malfunction_data")["malfunction_rate"]));
		
		float speed = p::extract<float>(agents[i].attr("speed_data")["speed"]);
		float position_fraction = p::extract<float>(agents[i].attr("speed_data")["position_fraction"]);
		int exit_action = p::extract<float>(agents[i].attr("speed_data")["transition_action_on_cellexit"]);
		pair<int, int> exit_loc = initial;
		int exit_heading;
		if (exit_action == 1) {
			exit_heading = heading - 1;
			if (exit_heading < 0)
				exit_heading = 3;
		}
		else if(exit_action == 3) {
			exit_heading = heading + 1;
			if (exit_heading > 3)
				exit_heading = 0;
		}
		else if (exit_action == 2) {
			exit_heading = heading;
		}
		else {
			exit_heading = -1;
		}

        this->agents_all[i].agent_id = i;
        this->agents_all[i].initial_location = ml.linearize_coordinate(initial);
        this->agents_all[i].goal_location = ml.linearize_coordinate(goal);
        this->agents_all[i].position = -1;
        this->agents_all[i].heading = heading;
        this->agents_all[i].status = status;
        this->agents_all[i].malfunction_left = malfunction;
        this->agents_all[i].next_malfunction = next_malfunction;
        this->agents_all[i].malfunction_rate = malfunction_rate;
        this->agents_all[i].speed = speed;
        this->agents_all[i].position_fraction = position_fraction;
        this->agents_all[i].exit_heading = exit_heading;

	}


}

void AgentsLoader::updateAgents(const FlatlandLoader &ml, p::object agents)
{
    // new_malfunction_agents.clear();
    new_agents.clear();
    num_active_agents = 0;
    for (int i = 0; i < num_of_agents_all; i++)
    {
        int new_status = p::extract<int>(agents[i].attr("status"));
        int malfunction_left = p::extract<int>(p::long_(agents[i].attr("malfunction_data")["malfunction"]));
        if (new_status == 1)
            num_active_agents++;
        if (agents_all[i].status == 0 && new_status == 1)
            new_agents.push_back(i); // agent i just starts to move
        else if (agents_all[i].status == 0 && agents_all[i].malfunction_left == 0 && malfunction_left > 0)
            new_agents.push_back(i); // agent i is about to move, but trapped by malfunction

        agents_all[i].status = new_status;
        if (agents_all[i].status <= 1)  // ready to depart (0) or active (1)
        { // update malfunction information
            if (agents_all[i].malfunction_left == 0 && malfunction_left > 0)
                new_malfunction_agents.push_back(i);
            agents_all[i].malfunction_left = malfunction_left;
            agents_all[i].next_malfunction = p::extract<int>(p::long_(agents[i].attr("malfunction_data")["next_malfunction"]));
            agents_all[i].malfunction_rate = p::extract<float>(p::long_(agents[i].attr("malfunction_data")["malfunction_rate"]));
        }

        // update position information
        if (agents_all[i].status == 1) // active
        {
            p::tuple iniTuple(agents[i].attr("position"));
            agents_all[i].position = ml.linearize_coordinate(p::extract<int>(p::long_(iniTuple[0])),
                    p::extract<int>(p::long_(iniTuple[1])));
            agents_all[i].heading = p::extract<int>(p::long_(agents[i].attr("direction")));
            agents_all[i].position_fraction = p::extract<float>(agents[i].attr("speed_data")["position_fraction"]);
            int exit_action = int(p::extract<float>(agents[i].attr("speed_data")["transition_action_on_cellexit"]));
            if (exit_action == 1) {
                agents_all[i].exit_heading = agents_all[i].heading - 1;
                if (agents_all[i].exit_heading < 0)
                    agents_all[i].exit_heading = 3;
            }
            else if (exit_action == 3) {
                agents_all[i].exit_heading = agents_all[i].heading + 1;
                if (agents_all[i].exit_heading > 3)
                    agents_all[i].exit_heading = 0;
            }
            else if (exit_action == 2) {
                agents_all[i].exit_heading = agents_all[i].heading;
            }
            else {
                agents_all[i].exit_heading = -1;
            }
        }
        else if (agents_all[i].status >= 2) // done (2) or done removed (3)
        {
            agents_all[i].position = agents_all[i].goal_location;
        }
    }
}


/*void AgentsLoader::updateAgents(p::object agents) {
	for (int i = 0; i < num_of_agents_all; i++) {
		pair<int, int> initial;
		pair<int, int> goal;
		bool activate = p::extract<int>(agents[i].attr("status")) == 1;
		if (activate) {
			p::tuple iniTuple(agents[i].attr("initial_position"));
			initial.first = p::extract<int>(p::long_(iniTuple[0]));
			initial.second = p::extract<int>(p::long_(iniTuple[1]));
		}

		p::tuple goalTuple(agents[i].attr("target"));
		goal.first = p::extract<int>(p::long_(goalTuple[0]));
		goal.second = p::extract<int>(p::long_(goalTuple[1]));
		int heading = p::extract<int>(p::long_(agents[i].attr("initial_direction")));
		//this->initial_locations_all[i]=initial;
		//this->goal_locations_all[i]=goal;
		//this->headings_all[i]=heading;

		int malfunction = p::extract<int>(p::long_(agents[i].attr("malfunction_data")["malfunction"]));
		int next_malfunction = p::extract<int>(p::long_(agents[i].attr("malfunction_data")["next_malfunction"]));
		float malfunction_rate = p::extract<float>(p::long_(agents[i].attr("malfunction_data")["malfunction_rate"]));
		
		float speed = p::extract<float>(agents[i].attr("speed_data")["speed"]);
		float position_fraction = p::extract<float>(agents[i].attr("speed_data")["position_fraction"]);
		int exit_action = p::extract<int>(agents[i].attr("speed_data")["transition_action_on_cellexit"]);
		pair<int, int> exit_loc = initial;
		int exit_heading;
		if (exit_action == 1) {
			exit_heading = heading - 1;
			if (exit_heading < 0)
				exit_heading = 3;
		}
		else if (exit_action == 3) {
			exit_heading = heading + 1;
			if (exit_heading > 3)
				exit_heading = 0;
		}
		else if (exit_action == 2) {
			exit_heading = heading;
		}
		else {
			exit_heading = -1;
		}

		Agent a;
		a.agent_id = i;
		a.initial_location = initial;
		a.goal_location = goal;
		a.position = initial;
		a.heading = heading;
		a.status = activate;
		a.malfunction_left = malfunction;
		a.next_malfunction = next_malfunction;
		a.malfunction_rate = malfunction_rate;
		a.speed = speed;
		a.position_fraction = position_fraction;
		a.exit_heading = exit_heading;

		this->agents_all[i]=a;

	}


}*/


void AgentsLoader::printAllAgentsInitGoal () const {
  cout << "AGENTS:" << endl;
    cout<<"number of agents: "<<num_of_agents_all<<endl;
  for (int i=0; i<num_of_agents_all; i++) {
    cout << "Agent" << i << " : " ;
      cout << "Initial: " << agents_all[i].initial_location << ", "
           << "Goal_location: " << agents_all[i].goal_location << ", "
           << "Distance: " << agents_all[i].distance_to_goal << ", "
           << "Status: " << agents_all[i].status << ", Heading: " << agents_all[i].heading
		<< ", malfunction_left: " << agents_all[i].malfunction_left
		<< ", next_malfuntion: " << agents_all[i].next_malfunction
		<< ", speed: " << agents_all[i].speed
		<< ", position_fraction: " << agents_all[i].position_fraction<<endl;
  }
  cout << endl;
}

void AgentsLoader::printCurrentAgentsInitGoal () const {
    cout << "AGENTS:" << endl;
    cout<<"number of agents: "<<num_of_agents<<endl;
    for (int i=0; i<num_of_agents; i++) {
        cout << "Agent" << i << " : " ;
        cout << "Initial: " << agents[i]->initial_location << ","
             << "Goal_location: " << agents[i]->goal_location << ","
             << "Status: " << agents[i]->status << ", Heading: " << agents[i]->heading
             << ", malfunction_left: " << agents[i]->malfunction_left
             << ", next_malfuntion: " << agents[i]->next_malfunction
             << ", speed: " << agents[i]->speed
             << ", position_fraction: " << agents[i]->position_fraction<<endl;
    }
    cout << endl;
}

AgentsLoader::~AgentsLoader() {
  // vectors are on stack, so they are freed automatically
}

// create an empty object
AgentsLoader::AgentsLoader() {
  num_of_agents_all = 0;
}

/*// returns the agents' ids if they occupy [row,col] (first for start, second for goal)
pair<int, int> AgentsLoader::agentStartOrGoalAt(int row, int col) {
  int f = -1;
  int s = -1;
  for (vector< pair<int, int> >::iterator it = initial_locations.begin(); it != initial_locations.end(); ++it)
    if ( it->first == row && it->second == col )
      f = std::distance(initial_locations.begin(), it);
  for (vector< pair<int, int> >::iterator it = goal_locations.begin(); it != goal_locations.end(); ++it)
    if ( it->first == row && it->second == col )
      s = std::distance(goal_locations.begin(), it);
  return make_pair(f, s);
}


void AgentsLoader::clearLocationFromAgents(int row, int col) {
  pair<int, int> idxs = agentStartOrGoalAt(row, col);
  if ( idxs.first != -1 ) {  // remove the agent who's start is at [row,col]
    initial_locations.erase( initial_locations.begin() + idxs.first );
    goal_locations.erase ( goal_locations.begin() + idxs.first );
    num_of_agents--;
  }
  idxs = agentStartOrGoalAt(row, col);
  if ( idxs.second != -1 ) {  // remove the agent who's goal is at [row,col]
    initial_locations.erase( initial_locations.begin() + idxs.second );
    goal_locations.erase( goal_locations.begin() + idxs.second );
    num_of_agents--;
  }
}


// add an agent
void AgentsLoader::addAgent(int start_row, int start_col, int goal_row, int goal_col) {
  this->initial_locations.push_back(make_pair(start_row, start_col));
  this->goal_locations.push_back(make_pair(goal_row, goal_col));
  num_of_agents++;
}*/



// agent_priority_strategy:
// 0: keep the original ordering
// 1: prefer max speed then max distance
// 2: prefer min speed then max distance
// 3: prefer max speed then min distance
// 4: prefer min speed then min distance
// 5: prefer different start locations then max speed then max distance
/*void AgentsLoader::generateAgentOrder(int agent_priority_strategy)
{
    if (agent_priority_strategy == 5)
    {
        // decide the agent priority for agents at the same start location
        boost::unordered_map<int, vector<int>> start_locations; // map the agents to their start locations
        for (int i = 0; i < num_of_agents_all; i++)
            start_locations[agents_all[i].position].push_back(i);
        for (auto& agents : start_locations)
        {
            quickSort(agents.second, 0, agents.second.size() - 1, agent_priority_strategy);
            for (int i = 0; i < (int)agents.second.size(); i++)
            {
                agents_all[agents.second[i]].priority = i;
            }
        }
    }

    // sort the agents
    vector<int> agent_order(num_of_agents_all);
    for (int i = 0; i < num_of_agents_all; i++)
        agent_order[i] = i;
    if (agent_priority_strategy != 0)
        quickSort(agent_order, 0, num_of_agents_all - 1, agent_priority_strategy);
    unplanned_agents = list<int>(agent_order.begin(), agent_order.end());
}*/

void AgentsLoader::updateConstraintTable()
{
    constraintTable.reset();
    for (int i = 0; i < (int)paths_all.size(); i++)
    {
        bool succ = constraintTable.insert_path(i, paths_all[i]);
        assert(succ);
    }
}

void AgentsLoader::quickSort(vector<int>& agent_order, int low, int high, int agent_priority_strategy)
{
    if (low >= high)
        return;
    auto& pivot = agents_all[agent_order[high]];    // pivot
    int i = low;  // Index of smaller element
    for (int j = low; j <= high - 1; j++)
    {
        // If current element is smaller than or equal to pivot
        if (compareAgent(agents_all[agent_order[j]], pivot, agent_priority_strategy))
        {
            std::swap(agent_order[i], agent_order[j]);
            i++;    // increment index of smaller element
        }
    }
    std::swap(agent_order[i], agent_order[high]);

    quickSort(agent_order, low, i - 1, agent_priority_strategy);  // Before i
    quickSort(agent_order, i + 1, high, agent_priority_strategy); // After i
}

/*void AgentsLoader::updateToBePlannedAgents(int _num_of_agents)
{
    if (unplanned_agents.empty())
    {
        this->num_of_agents = 0;
        return;
    }
    this->num_of_agents = min(_num_of_agents, (int)unplanned_agents.size());
    agents.resize(num_of_agents);
    auto p = unplanned_agents.begin();
    cout << "Agents ids: ";
    for (int i = 0; i < num_of_agents; i++)
    {
        cout << *p << ",";
        agents[i] = &agents_all[*p];
        ++p;
    }
    cout << endl;
}*/

void AgentsLoader::sampleAgents(int _num_of_agents, int iteration, int num_instances, bool deletePath)
{

    this->num_of_agents = _num_of_agents;
    agents.resize(num_of_agents);
    cout << "Agents ids: ";
    int start = agents_all.size()/num_instances * (iteration-1);
    for (int i = 0; i < num_of_agents; i++)
    {
        int agent = (start+i)%agents_all.size();
        cout << agent << ",";
        agents[i] = &agents_all[agent];
        if (deletePath){
            constraintTable.delete_path(agent, paths_all[agent]);
        }
    }
    cout << endl;
}

void AgentsLoader::recoverAgents(int _num_of_agents, int iteration, int num_instances)
{

    this->num_of_agents = _num_of_agents;
    int start = agents_all.size()/num_instances * (iteration-1);
    for (int i = 0; i < num_of_agents; i++)
    {
        int agent = (start+i)%agents_all.size();
        constraintTable.insert_path(agent, paths_all[agent]);
    }
}

/*bool AgentsLoader::addPaths(const vector<Path*>& new_paths)
{
    assert((int)new_paths.size() == num_of_agents);
    list<int> giveup_agents;
    for (int i = 0; i < num_of_agents; i++)
    {
        auto a = unplanned_agents.front();
        unplanned_agents.pop_front();

        if (new_paths[i] == nullptr)
        {
            giveup_agents.push_back(a);
            continue;
        }
        assert(paths_all[a].empty());
        if(new_paths[i]->empty())
            num_of_dead_agents++;
        paths_all[a] = *new_paths[i];
        // add the path to the constraint table
        if(!constraintTable.insert_path(a, paths_all[a]))
            return false;
        makespan = max(makespan, (int)paths_all[a].size() - 1);
    }
    unplanned_agents.splice(unplanned_agents.begin(), giveup_agents);
    return true;
}*/

void AgentsLoader::computeHeuristics(const FlatlandLoader* ml, std::unordered_map<int,vector<hvals>>& existing_heuristics)
{
    sum_of_distances = 0;
    for (auto& agent : agents_all) {
        int init_loc = agent.initial_location;
        int goal_loc = agent.goal_location;

        if (existing_heuristics.count(goal_loc)){
            agent.heuristics = &existing_heuristics[goal_loc];

        }
        else{
            existing_heuristics[goal_loc] = vector<hvals>();
            ComputeHeuristic<FlatlandLoader> ch(init_loc, goal_loc, ml, agent.heading);
            ch.getHVals(existing_heuristics[goal_loc]);
            agent.heuristics = &existing_heuristics[goal_loc];
        }

        agent.distance_to_goal = (*agent.heuristics)[init_loc].get_hval(agent.heading);
        sum_of_distances += agent.distance_to_goal;
    }
}

AgentsLoader* AgentsLoader::clone(){
    AgentsLoader* new_al = new AgentsLoader();
    new_al->num_of_agents_all =  this->num_of_agents_all;
    ///this->heuristics.resize(num_of_agents_all);
    new_al->agents_all =  this->agents_all;
    new_al->paths_all =  this->paths_all;
    new_al->constraintTable = this->constraintTable;
    new_al->sum_of_distances = this->sum_of_distances;
    return new_al;

}