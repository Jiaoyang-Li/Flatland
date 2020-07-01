//=======================================================================

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
	os << "Initial: (" << agent.initial_location.first << "," << agent.initial_location.second << "),"
		<< "Goal_location: (" << agent.goal_location.first << "," << agent.goal_location.second << "),"
		<< "Activate: " << agent.activate << ", Heading: " << agent.heading
		<< ", malfunction_left: " << agent.malfunction_left
		<< ", next_malfuntion: " << agent.next_malfuntion
		<< ", speed: " << agent.speed
		<< ", position_fraction: " << agent.position_fraction
        ;
    return os;
}

AgentsLoader::AgentsLoader(p::object agents) {
	int agentsNum = p::len(agents);
	this->num_of_agents_all = agentsNum;
	///this->heuristics.resize(num_of_agents_all);
    this->agents_all.resize(num_of_agents_all);
    this->blocked_paths.resize(num_of_agents_all);
    //this->initial_locations_all.resize(num_of_agents_all);
    //this->goal_locations_all.resize(num_of_agents_all);
    //this->headings_all.resize(num_of_agents_all);
	for (int i = 0; i < num_of_agents_all; i++) {
		pair<int, int> initial;
		pair<int, int> goal;
		bool activate = p::extract<int>(agents[i].attr("status")) == 1;
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

		int malfunction = 0;
		int next_malfunction = 0;
		float malfunction_rate = 0;
		
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


        this->agents_all[i].initial_location = initial;
        this->agents_all[i].goal_location = goal;
        this->agents_all[i].position = initial;
        this->agents_all[i].heading = heading;
        this->agents_all[i].activate = activate;
        this->agents_all[i].malfunction_left = malfunction;
        this->agents_all[i].next_malfuntion = next_malfunction;
        this->agents_all[i].malfunction_rate = malfunction_rate;
        this->agents_all[i].speed = speed;
        this->agents_all[i].position_fraction = position_fraction;
        this->agents_all[i].exit_heading = exit_heading;

	}


}

void AgentsLoader::updateAgents(p::object agents) {
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
		a.initial_location = initial;
		a.goal_location = goal;
		a.position = initial;
		a.heading = heading;
		a.activate = activate;
		a.malfunction_left = malfunction;
		a.next_malfuntion = next_malfunction;
		a.malfunction_rate = malfunction_rate;
		a.speed = speed;
		a.position_fraction = position_fraction;
		a.exit_heading = exit_heading;

		this->agents_all[i]=a;

	}


}

AgentsLoader::AgentsLoader(const string& fname, const MapLoader &ml, int agentsNum = 0){
  string line;

  ifstream myfile (fname.c_str());

  if (myfile.is_open()) {
    getline (myfile,line);
    char_separator<char> sep(",");
    tokenizer< char_separator<char> > tok(line, sep);
    tokenizer< char_separator<char> >::iterator beg=tok.begin();
    this->num_of_agents_all = atoi ( (*beg).c_str() );
    //this->heuristics.resize(num_of_agents_all);
      this->agents_all.resize(num_of_agents_all);
      this->blocked_paths.resize(num_of_agents_all);
      //this->initial_locations_all.resize(num_of_agents_all);
      //this->goal_locations_all.resize(num_of_agents_all);
      //this->headings_all.resize(num_of_agents_all);
    //    cout << "#AG=" << num_of_agents_all << endl;
    for (int i=0; i<num_of_agents_all; i++) {
      getline (myfile, line);
      tokenizer< char_separator<char> > col_tok(line, sep);
      tokenizer< char_separator<char> >::iterator c_beg=col_tok.begin();
      pair<int,int> curr_pair;
      // read start [row,col] for agent i
      curr_pair.first = atoi ( (*c_beg).c_str() );
      c_beg++;
      curr_pair.second = atoi ( (*c_beg).c_str() );
      //      cout << "AGENT" << i << ":   START[" << curr_pair.first << "," << curr_pair.second << "] ; ";
      this->agents_all[i].initial_location = curr_pair;
      // read goal [row,col] for agent i
      c_beg++;
      curr_pair.first = atoi ( (*c_beg).c_str() );
      c_beg++;
      curr_pair.second = atoi ( (*c_beg).c_str() );
      //      cout << "GOAL[" << curr_pair.first << "," << curr_pair.second << "]" << endl;
      this->agents_all[i].goal_location = curr_pair;
	  this->agents_all[i].heading = -1;

      // read max velocity and accelration for agent i
     /* c_beg++;
      this->max_v.push_back(atof((*c_beg).c_str()));
      c_beg++;
      this->max_w.push_back(atof((*c_beg).c_str()));
      c_beg++;
      this->max_a.push_back(atof((*c_beg).c_str()));*/
    }
    myfile.close();
  } 
  else if(agentsNum > 0)//Generate agents randomly
  {
	  this->num_of_agents_all = agentsNum;
	  //this->heuristics.resize(num_of_agents_all);
      this->agents_all.resize(num_of_agents_all);
      this->blocked_paths.resize(num_of_agents_all);
      //this->initial_locations_all.resize(num_of_agents_all);
      //this->goal_locations_all.resize(num_of_agents_all);
      //this->headings_all.resize(num_of_agents_all);
	  vector<bool> starts(ml.rows * ml.cols, false);
	  vector<bool> goals(ml.rows * ml.cols, false);
	  // Choose random start locations
	  for (int k = 0; k < agentsNum; k++)
	  {
		  int x = rand() % ml.rows, y = rand() % ml.cols;
		  int start = x * ml.cols +y;
		  if (!ml.my_map[start] && !starts[start])
		  {
				// update start
				this->agents_all[k].initial_location = make_pair(x,y);
				starts[start] = true;

				// random walk
				int loc = start;
				bool* temp_map = new bool[ml.rows * ml.cols];
				for (int walk = 0; walk < RANDOM_WALK_STEPS; walk++)
				{
					int directions[] = {0, 1, 2, 3, 4};
					random_shuffle(directions, directions + 5);
					int i = 0;
					for(; i< 5; i++)
					{
						int next_loc = loc + ml.moves_offset[directions[i]];
						if (0 <= next_loc && next_loc < ml.rows * ml.cols &&! ml.my_map[next_loc])
						{
							loc = next_loc;
							break;
						}
					}
					if (i == 5)
					{
						cout << "--------------ERROR!-----------------" << endl;
						// system("pause");
					}
				}
				// find goal
				bool flag = false;
				int goal = loc;
				while (!flag)
				{
					int directions[] = { 0, 1, 2, 3, 4 };
					random_shuffle(directions, directions + 5);
					int i = 0;
					for (; i< 5; i++)
					{
						int next_loc = goal + ml.moves_offset[directions[i]];
						if (0 <= next_loc && next_loc < ml.rows * ml.cols && !ml.my_map[next_loc])
						{
							goal = next_loc;
							break;
						}
					}
					if (i == 5)
					{
						cout << "--------------ERROR!-----------------" << endl;
						// system("pause");
					}
					flag = true;
					if (goals[goal])
						flag = false;
				}
				//update goal
				this->agents_all[k].goal_location = make_pair(goal / ml.cols, goal % ml.cols);
				goals[goal] = true;
				this->agents_all[k].heading = -1;

				// update others
				/*this->max_v.push_back(1);
				this->max_w.push_back(1);
				this->max_a.push_back(1);*/
		  }
		  else
		  {
			  k--;
		  }
	  }
	  saveToFile(fname);
  }
  else
  {
	  cerr << "Agent file " << fname << " not found." << std::endl;
	  exit(10);
  }
}

void AgentsLoader::printAllAgentsInitGoal () const {
  cout << "AGENTS:" << endl;
    cout<<"number of agents: "<<num_of_agents_all<<endl;
  for (int i=0; i<num_of_agents_all; i++) {
    cout << "Agent" << i << " : " ;
      cout << "Initial: (" << agents_all[i].initial_location.first << "," << agents_all[i].initial_location.second << "),"
		<< "Goal_location: (" << agents_all[i].goal_location.first << "," << agents_all[i].goal_location.second << "),"
		<< "Activate: " << agents_all[i].activate << ", Heading: " << agents_all[i].heading
		<< ", malfunction_left: " << agents_all[i].malfunction_left
		<< ", next_malfuntion: " << agents_all[i].next_malfuntion
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
        cout << "Initial: (" << agents[i]->initial_location.first << "," << agents[i]->initial_location.second << "),"
             << "Goal_location: (" << agents[i]->goal_location.first << "," << agents[i]->goal_location.second << "),"
             << "Activate: " << agents[i]->activate << ", Heading: " << agents[i]->heading
             << ", malfunction_left: " << agents[i]->malfunction_left
             << ", next_malfuntion: " << agents[i]->next_malfuntion
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

void AgentsLoader::saveToFile(const std::string& fname) {
  ofstream myfile;
  myfile.open(fname);
  myfile << num_of_agents_all << endl;
  for (int i = 0; i < num_of_agents_all; i++)
    myfile << agents_all[i].initial_location.first << ","
            << agents_all[i].initial_location.second << ","
           << agents_all[i].goal_location.first << ","
           << agents_all[i].goal_location.second << ","
		   /*<< max_v[i] << "," << max_a[i] << "," << max_w[i] << ","*/  << endl;
  myfile.close();
}


void AgentsLoader::generateAgentOrder() // sort the agents by their speeds in decreasing order
{
    agent_order.resize(num_of_agents_all);
    for (int i = 0; i < num_of_agents_all; i++)
        agent_order[i] = i;
    quickSort(0, num_of_agents_all - 1);
}


void AgentsLoader::quickSort(int low, int high)
{
    if (low >= high)
        return;
    auto pivot = agents_all[agent_order[high]].speed;    // pivot
    int i = low;  // Index of smaller element
    for (int j = low; j <= high - 1; j++)
    {
        // If current element is larger than or equal to pivot
        if (agents_all[agent_order[j]].speed >= pivot)
        {
            std::swap(agent_order[i], agent_order[j]);
            i++;    // increment index of smaller element
        }
    }
    std::swap(agent_order[i], agent_order[high]);

    quickSort(low, i - 1);  // Before i
    quickSort(i + 1, high); // After i
}

void AgentsLoader::updateToBePlannedAgents(int _num_of_agents)
{
    this->num_of_agents = min(_num_of_agents, num_of_agents_all - num_of_agents_finished);
    if (num_of_agents <= 0)
        return;
    agents.resize(num_of_agents);
    for (int i = 0; i < num_of_agents; i++)
    {
        agents[i] = &agents_all[agent_order[num_of_agents_finished + i]];
    }
}

void AgentsLoader::addPaths(const vector<Path*>& new_paths)
{
    assert((int)new_paths.size() == num_of_agents);
    for (int i = 0; i < num_of_agents; i++)
    {
        assert(blocked_paths[agent_order[num_of_agents_finished + i]].empty());
        assert(new_paths[i]->size() > 0);
        blocked_paths[agent_order[num_of_agents_finished + i]] = *new_paths[i];
    }
    num_of_agents_finished += num_of_agents;
}