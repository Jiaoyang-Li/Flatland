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
#include "compute_heuristic.h"

using namespace std;

struct Agent {
	pair<int, int> initial_location;
	pair<int, int> goal_location;
	pair<int, int> position;
    vector<hvals> heuristics;
	bool activate = true;
	int heading = -1;
	int malfunction_left = 0;
	int next_malfuntion = 0;
	float malfunction_rate = 0;
	float speed = 1.0;
	float position_fraction = 0.0;
	int exit_heading = -1;
};

class AgentsLoader {
public:
    int num_of_agents;
    vector<Agent*> agents;
    vector<Path> paths_all; // already planned paths are viewed as obstacles for future iterations
    //vector< pair<int, int> > initial_locations;
    //vector< pair<int, int> > goal_locations;
    //vector<int> headings;
    // vector<double> max_v;  // entry [i] is the max translational velocity for agent i
    // vector<double> max_w;  // entry [i] is the max rotational velocity for agent i
    // vector<double> max_a;  // entry [i] is the max accelration for agent i
    AgentsLoader(const std::string& fname, const MapLoader &ml, int agentsNum);
    AgentsLoader();
    AgentsLoader(boost::python::object agents);
    void updateAgents(boost::python::object agents);
    // void addAgent ( int start_row, int start_col, int goal_row, int goal_col );
    void printAgentsInitGoal ();
    void saveToFile(const std::string& fname);
    // pair<int, int> agentStartOrGoalAt(int row, int col);
    // void clearLocationFromAgents(int row, int col);
    ~AgentsLoader();

    void generateAgentOrder();
    void updateToBePlannedAgents() { updateToBePlannedAgents(num_of_agents_all); };
    void updateToBePlannedAgents(int num_of_agents);
    void addPaths(const vector<Path*>& paths);

private:
    int num_of_agents_finished = 0;
    vector<int> agent_order;
    int num_of_agents_all;
    vector<Agent> agents_all;
    //vector< pair<int, int> > initial_locations_all;
    //vector< pair<int, int> > goal_locations_all;
    //vector<int> headings_all;
    // vector< vector<hvals> > heuristics;  // [agent_id][loc]

    void quickSort(int low, int high);
};

#endif
