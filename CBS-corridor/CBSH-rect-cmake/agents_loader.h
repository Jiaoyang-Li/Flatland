// Load's agents' init and goal states.
// First line: number of agents
// Second line and onward, (x_init,y_init),(x_goal,y_goal) of each agent (one per line)

#ifndef AGENTSLOADER_H
#define AGENTSLOADER_H

#include <string>
#include <vector>
#include <utility>
#include "map_loader.h"
#include <boost/python.hpp>


using namespace std;

struct Agent {
	pair<int, int> initial_location;
	pair<int, int> goal_location;
	pair<int, int> position;

	bool activate = true;
	int heading = -1;
	int malfunction_left = 0;
	int next_malfuntion = 0;
	float malfunction_rate = 0;
	float speed = 1.0;
	float position_fraction = 0.0;
	int exit_heading;
};

class AgentsLoader {
 public:
  int num_of_agents;
  vector<Agent> agents;
  vector< pair<int, int> > initial_locations;
  vector< pair<int, int> > goal_locations;
  vector<int> headings;
  vector<double> max_v;  // entry [i] is the max translational velocity for agent i
  vector<double> max_w;  // entry [i] is the max rotational velocity for agent i
  vector<double> max_a;  // entry [i] is the max accelration for agent i
  AgentsLoader(const std::string fname, const MapLoader &ml, int agentsNum);
  AgentsLoader();
  AgentsLoader(boost::python::object agents);
  void updateAgents(boost::python::object agents);
  void addAgent ( int start_row, int start_col, int goal_row, int goal_col );
  void printAgentsInitGoal ();
  void saveToFile(const std::string fname);
  pair<int, int> agentStartOrGoalAt(int row, int col);
  void clearLocationFromAgents(int row, int col);
  ~AgentsLoader();
};

#endif
