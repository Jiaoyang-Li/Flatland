/*
 * @author: Team An_old_driver
 * @created: 09-2020
 * Copyright (c) 2020 The University of Southern California. All Rights Reserved.
 * Copyrights licensed under an Academic/non-profit use license.
 * See the accompanying LICENSE file for terms.
*/

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
	int initial_location;
	int goal_location;
	int position;
    vector<hvals>* heuristics = NULL;
    int agent_id;
	int status = 0;
	int heading = -1;
	int malfunction_left = 0;
	int next_malfunction = 0;
	float malfunction_rate = 0;
	float speed = 1.0;
	float position_fraction = 0.0;
	int exit_heading = -1;
	int distance_to_goal = MAX_COST;
	int priority = 0;

};

class AgentsLoader {
public:
    int num_of_agents;
    vector<Agent*> agents;
    ConstraintTable constraintTable; // store already planned paths, which are viewed as obstacles for future iterations

    vector<Agent> agents_all;
    vector<Path> paths_all;
    int makespan = 0; // the makepsan of the paths in paths_all

    list<int> new_malfunction_agents; // agents that have just got malfunction
    list<int> new_agents; // agents that have just appear on the map
    list<int> unplanned_agents;
    int num_active_agents = 0;
    int sum_of_distances = 0;

    AgentsLoader();
    AgentsLoader(const FlatlandLoader &ml, boost::python::object agents);
    void updateAgents(const FlatlandLoader &ml, boost::python::object agents);
    void updateConstraintTable();
    // void addAgent ( int start_row, int start_col, int goal_row, int goal_col );
    void printAllAgentsInitGoal () const;
    void printCurrentAgentsInitGoal () const;
    void printPath(int i) const
    {
        cout << "Agent " << i << ": ";
        for (const auto & entry : paths_all[i])
            cout << entry.location << ",";
        cout << endl;
    }
    // pair<int, int> agentStartOrGoalAt(int row, int col);
    // void clearLocationFromAgents(int row, int col);
    ~AgentsLoader();

    //void generateAgentOrder(int agent_priority_strategy);
    //void updateToBePlannedAgents() { updateToBePlannedAgents(num_of_agents_all); };
    //void updateToBePlannedAgents(int num_of_agents);
    //bool addPaths(const vector<Path*>& paths);
    int getNumOfUnplannedAgents() const { return (int)unplanned_agents.size(); }
    int getNumOfAllAgents() const { return num_of_agents_all; }
    int getNumOfDeadAgents() const { return num_of_dead_agents; }
    void sampleAgents(int _num_of_agents, int iteration, int num_instances, bool deletePath = false);
    void recoverAgents(int _num_of_agents, int iteration, int num_instances);
    AgentsLoader* clone();


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

    void computeHeuristics(const FlatlandLoader* ml,std::unordered_map<int,vector<hvals>>& existing_heuristics);
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

    static inline bool compareAgent(const Agent& a1, const Agent& a2, int agent_priority_strategy)
    {  // return true if a1 < a2
        if (agent_priority_strategy == 1)    // 1: prefer max speed then max distance
        {
            if (a1.speed == a2.speed)
            {
                if (a1.distance_to_goal == a2.distance_to_goal)
                {
                    return a1.agent_id <= a2.agent_id;
                }
                return a1.distance_to_goal >= a2.distance_to_goal;
            }
            return a1.speed >= a2.speed;
        }
        else if (agent_priority_strategy == 2)    // 2: prefer min speed then max distance
        {
            if (a1.speed == a2.speed)
            {
                if (a1.distance_to_goal == a2.distance_to_goal)
                {
                    return a1.agent_id <= a2.agent_id;
                }
                return a1.distance_to_goal >= a2.distance_to_goal;
            }
            return a1.speed <= a2.speed;
        }
        else if (agent_priority_strategy == 3)    // 3: prefer max speed then min distance
        {
            if (a1.speed == a2.speed)
            {
                if (a1.distance_to_goal == a2.distance_to_goal)
                {
                    return a1.agent_id <= a2.agent_id;
                }
                return a1.distance_to_goal <= a2.distance_to_goal;
            }
            return a1.speed >= a2.speed;
        }
        else if (agent_priority_strategy == 4)    // 4: prefer min speed then min distance
        {
            if (a1.speed == a2.speed)
            {
                if (a1.distance_to_goal == a2.distance_to_goal)
                {
                    return a1.agent_id <= a2.agent_id;
                }
                return a1.distance_to_goal <= a2.distance_to_goal;
            }
            return a1.speed <= a2.speed;
        }
        else if (agent_priority_strategy == 5)    // 5: prefer different start locations then max speed then max distance
        {
            if (a1.priority == a2.priority)
            {
                if (a1.speed == a2.speed)
                {
                    if (a1.distance_to_goal == a2.distance_to_goal)
                    {
                        return a1.agent_id <= a2.agent_id;
                    }
                    return a1.distance_to_goal >= a2.distance_to_goal;
                }
                return a1.speed >= a2.speed;
            }
            return a1.priority < a2.priority;
        }
        else if (agent_priority_strategy == 6)    // 6: prefer same start locations then max speed then min distance
        {
            if (a1.priority == a2.priority)
            {
                if (a1.speed == a2.speed)
                {
                    if (a1.distance_to_goal == a2.distance_to_goal)
                    {
                        return a1.agent_id <= a2.agent_id;
                    }
                    return a1.distance_to_goal <= a2.distance_to_goal;
                }
                return a1.speed >= a2.speed;
            }
            return a1.priority < a2.priority;
        }
        else
            return true;    // keep the original ordering
    }

private:
    int num_of_dead_agents = 0;
    int num_of_agents_all;

    void quickSort(vector<int>& agent_order, int low, int high, int agent_priority_strategy);
};

#endif
