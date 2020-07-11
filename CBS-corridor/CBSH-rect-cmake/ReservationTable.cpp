#include "ReservationTable.h"
#include <iostream>
#include <memory>

ReservationTable::ReservationTable(int mapSize, int max_malfunction, bool ignoreFinishedAgent) {
	this->ignoreFinishedAgent = ignoreFinishedAgent;
	this->max_malfunction = max_malfunction;
	res_table = map_table(mapSize);
}

ReservationTable::ReservationTable(int mapSize, vector<vector<PathEntry>*>* paths, int exclude, int max_malfunction, bool ignoreFinishedAgent) {
	this->ignoreFinishedAgent = ignoreFinishedAgent;
	this->max_malfunction = max_malfunction;

	res_table = map_table(mapSize);

	addPaths(paths,exclude);
}

void ReservationTable::addPath(int agent_id, const Path& path)
{
	AgentStep* preStep = nullptr;
	for (int t = 0; t < path.size(); t++) {
		int loc = path[t].location;
        if (loc == -1)
            continue;
		if (!res_table.count(loc)) {
			res_table[loc] = timeline();
		}
		if (!res_table[loc].count(t)) {
			res_table[loc][t] = agentList();
		}
		

		res_table[loc][t][agent_id] = AgentStep(agent_id, loc, t);
		if (preStep != nullptr) {
			preStep->nextStep = &(res_table[loc][t][agent_id]);
		}
		res_table[loc][t][agent_id].preStep = preStep;
		preStep = &(res_table[loc][t][agent_id]);

		if (!ignoreFinishedAgent && t == path.size() - 1 && !path[t].malfunction) {
			goalTable[loc][agent_id] = t;
		}

		//for malfunction agent, hold the location for 5 timestep
		if (path[t].malfunction) {
			for (int i = 1; i <= this->max_malfunction; i++) {
				if (!res_table[loc].count(t+i)) {
					res_table[loc][t+i] = agentList();
				}
				res_table[loc][t+i][agent_id] = AgentStep(agent_id, loc, t+i);
				if (preStep != nullptr) {
					preStep->nextStep = &(res_table[loc][t+i][agent_id]);
				}
				res_table[loc][t+i][agent_id].preStep = preStep;
				preStep = &(res_table[loc][t+i][agent_id]);
			}
		}
	}
}

void ReservationTable::addPaths(const vector<Path*>* paths,int exclude) {
	for (int agent = 0; agent < paths->size(); agent++) {
		if (agent == exclude || (*paths)[agent]==nullptr)
			continue;
		addPath(agent, *(paths->at(agent)));
	}
}

void ReservationTable::addPaths(const vector<Path>& paths, int exclude)
{
    for (int agent = 0; agent < paths.size(); agent++) {
        if (agent == exclude)
            continue;
        addPath(agent, paths[agent]);
    }
}
void ReservationTable::deletePath(int agent_id, const vector<PathEntry>& path)
{
	for (int t = 0; t < path.size(); t++)
	{
		int loc = path[t].location;
		assert(res_table.count(loc) && res_table[loc].count(t));
		res_table[loc][t].erase(agent_id);
	}
}

OldConfList* ReservationTable::findConflict(int agent, int currLoc, int nextLoc, int currT,int kDelay) {
    auto* confs =  new OldConfList;
    int nextT = currT + 1;
    if(nextLoc == -1)
        return confs;
    //cout << "currloc " << currLoc << " nextloc " << nextLoc << endl;
//	if (currLoc == -1 && currT==0)
//	    kDelay = 0;
    if (res_table.count(nextLoc)) {
        //detect vertex conflict and k delay vertex conflict
        for (int k = -kDelay;  k <= kDelay; k++) {

            int t = nextT + k;

            if (res_table[nextLoc].count(t)) {
                agentList::iterator it;
                for (const auto& agents : res_table[nextLoc][t]) {
                    confs->push_back(std::make_shared<tuple<int, int, int, int, int,int>>(
                            k < 0 ? agents.second.agent_id : agent,
                            k < 0 ? agent : agents.second.agent_id,
                            nextLoc,
                            -1,
                            k < 0 ? t : nextT,
                            k >= 0 ? k : -k));

                }
            }


        }
        if (goalTable.count(nextLoc)) {
            goalAgentList::iterator it;
            for (it = goalTable[nextLoc].begin(); it != goalTable[nextLoc].end(); ++it) {

                if (nextT > it->second) {
                    confs->push_back(std::make_shared<tuple<int, int, int, int, int,int>>(
                            it->first, agent, nextLoc, -1, nextT, 0));

                }
            }
        }
        //detect edge conflict, we do not detect k delay edge conflict, because, every k delay edge conflict cause k delay vertex conflict.
        //every edge conflict cause two k delay vertex conflict, thus don't need to detect edge conflict when k is not 0.
        if (kDelay == 0) {
            if (res_table[nextLoc].count(currT)) {
                agentList::iterator it;
                for (it = res_table[nextLoc][currT].begin(); it != res_table[nextLoc][currT].end(); ++it) {

                    if (it->second.nextStep != nullptr && it->second.nextStep->loc == currLoc) {
                        confs->push_back(std::make_shared<tuple<int, int, int, int, int,int>>(
                                agent, it->second.agent_id, currLoc, nextLoc, nextT, 0));
                    }
                }
            }
        }
    }
    return confs;
}

int ReservationTable::countConflict(int agent, int currLoc, int nextLoc, int currT,int kDelay) {
    int nextT = currT + 1;
    if(nextLoc == -1)
        return 0;
    int count = 0;

    if (res_table.count(nextLoc)) {
        //detect vertex conflict and k delay vertex conflict
        for (int k = -kDelay;  k <= kDelay; k++) {

            int t = nextT + k;

            if (res_table[nextLoc].count(t)) {
               count++;
            }


        }

        //detect edge conflict, we do not detect k delay edge conflict, because, every k delay edge conflict cause k delay vertex conflict.
        //every edge conflict cause two k delay vertex conflict, thus don't need to detect edge conflict when k is not 0.
        if (kDelay == 0) {
            if (res_table[nextLoc].count(currT)) {
                agentList::iterator it;
                for (it = res_table[nextLoc][currT].begin(); it != res_table[nextLoc][currT].end(); ++it) {

                    if (it->second.nextStep != nullptr && it->second.nextStep->loc == currLoc) {
                       count++;
                    }

                }
            }
        }


    }
    return count;
}





