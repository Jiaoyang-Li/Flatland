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
#include "flat_map_loader.h"
#include "ConstraintTable.h"
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
	int distance_to_goal = MAX_COST;
};

class AgentsLoader {
public:
    int num_of_agents;
    vector<Agent*> agents;
    ConstraintTable constraintTable; // store already planned paths, which are viewed as obstacles for future iterations
    vector<Path> paths_all;

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
    void printAllAgentsInitGoal () const;
    void printCurrentAgentsInitGoal () const;
    void saveToFile(const std::string& fname);
    // pair<int, int> agentStartOrGoalAt(int row, int col);
    // void clearLocationFromAgents(int row, int col);
    ~AgentsLoader();

    void generateAgentOrder();
    void updateToBePlannedAgents() { updateToBePlannedAgents(num_of_agents_all); };
    void updateToBePlannedAgents(int num_of_agents);
    void addPaths(const vector<Path*>& paths, int kDelay);
    int getNumOfUnplannedAgents() const { return (int)unplanned_agents.size(); }
    int getNumOfAllAgents() const { return num_of_agents_all; }
    int getNumOfDeadAgents() const { return num_of_dead_agents; }
    boost::python::list outputPaths()   {
        boost::python::list result;
        for (const auto& path : paths_all)  {
            boost::python::list agentPath;
            for (const auto& state : path)
                agentPath.append(state.location);
            result.append(agentPath);
        }
        return result;
    }

    void computeHeuristics(const FlatlandLoader* ml);
    void printPaths() const
    {
        for (int i = 0; i < (int)paths_all.size(); i++)
        {
            std::cout << "Agent " << i << ": ";
            for (int t = 0; t < (int)paths_all[i].size(); t++)
                std::cout << t <<"(" << paths_all[i][t].location << ")->";
            std::cout << std::endl;
        }
    }
    Agent getAgent(int id){
        return agents_all[id];
    }
private:
    int num_of_dead_agents = 0;
    list<int> unplanned_agents;
    int num_of_agents_all;
    vector<Agent> agents_all;
    //vector< pair<int, int> > initial_locations_all;
    //vector< pair<int, int> > goal_locations_all;
    //vector<int> headings_all;
    // vector< vector<hvals> > heuristics;  // [agent_id][loc]

    void quickSort(vector<int>& agent_order, int low, int high);
};

#endif
