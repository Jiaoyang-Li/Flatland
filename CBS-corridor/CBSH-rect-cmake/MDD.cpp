#include "MDD.h"
#include "flat_map_loader.h"

#include <iostream>

//Build mdd with heading info in search engine
template<class Map>
bool MDD<Map>::buildMDD( ConstraintTable& constraints, int numOfLevels, SingleAgentICBS<Map> & solver)
{
	MDDNode* root = new MDDNode(-1, NULL); // Root
    root->heading = solver.start_heading;
    root->row = solver.start_location / solver.num_col;
    root->col = solver.start_location % solver.num_col;
    root->position_fraction = solver.al->agents[solver.agent_id]->position_fraction;
    root->malfunction_left = solver.al->agents[solver.agent_id]->malfunction_left;
    root->next_malfunction = solver.al->agents[solver.agent_id]->next_malfuntion;
	std::queue<MDDNode*> open;
	std::list<MDDNode*> closed;
	open.push(root);
	closed.push_back(root);
	levels.resize(numOfLevels);
	//cout << "start: " << solver.start_heading << endl;
	//cout << "goal: " << solver.goal_location << endl;

	while (!open.empty())
	{
		MDDNode* node = open.front();
		open.pop();
		// Here we suppose all edge cost equals 1
		if (node->level == numOfLevels - 1)
		{
            //cout << "loc: " << node->location << endl;

            levels[numOfLevels - 1].push_back(node);
			if(!open.empty())
			{
				while (!open.empty())
				{
					MDDNode* node = open.front();
					open.pop();
					//cout << "loc: " << node->location << " heading: " << node->heading<<" h "<< solver.my_heuristic[node->location].heading[node->heading] <<" "<< solver.my_heuristic[node->location].heading.count(node->heading)<< endl;

				}
				
				std::cerr << "Failed to build MDD!" << std::endl;
				exit(1);
			}
			break;
		}
		// We want (g + 1)+h <= f = numOfLevels - 1, so h <= numOfLevels - g. -1 because it's the bound of the children.
		double heuristicBound = numOfLevels - node->level - 2+ 0.001;

        vector<Transition> transitions;
        if(node->location == -1){
            Transition move;
            move.first = -1;
            move.second = 4;
            move.position_fraction = node->position_fraction;
            transitions.push_back(move);

            Transition move2;
            move2.first =  solver.start_location;
            move2.second = 4;
            move2.position_fraction = node->position_fraction;
            transitions.push_back(move2);

        }
        else if (node->position_fraction + solver.al->agents[solver.agent_id]->speed >= 0.97) {
            if (node->position_fraction == 0)
                transitions = solver.ml->get_transitions(node->location, node->heading, false);
            else
                transitions = solver.ml->get_transitions(node->location, node->heading, true);
        }
        else if (node->position_fraction == 0) {
            Transition move;
            move.first = node->location;
            move.second = 4;
            move.position_fraction = node->position_fraction;

            transitions.push_back(move);

            Transition move2;
            move2.first = node->location;
            move2.second = 4;
            move2.position_fraction = node->position_fraction + solver.al->agents[solver.agent_id]->speed;

            transitions.push_back(move2);
        }
        else { //<0.97 and po_frac not 0


            Transition move2;
            move2.first = node->location;
            move2.second = 4;
            move2.position_fraction = node->position_fraction + solver.al->agents[solver.agent_id]->speed;

            transitions.push_back(move2);


        }
		//cout << "current " << node->location << " heading " << node->heading << endl;
		for (const auto move : transitions)
		{
            float next_position_fraction = move.position_fraction;

            int new_heading;
			if (node->heading == -1) //heading == -1 means no heading info
				new_heading = -1;
			else
				if (move.second == 4) //move == 4 means wait
					new_heading = node->heading;
				else
					new_heading = move.second;
			int newLoc = move.first;
			//cout << "newLoc " << newLoc << " heading " << new_heading<<" h "<< solver.my_heuristic[newLoc].heading[new_heading] << endl;
			if(newLoc != -1 && !solver.my_heuristic[newLoc].heading.count(new_heading)) {
			    continue;
			}
			int new_h;
			if(newLoc == -1)
			    new_h = solver.my_heuristic[solver.start_location].heading[solver.start_heading];
            else
                new_h = solver.my_heuristic[newLoc].heading[new_heading];

			if ( (new_h < heuristicBound) &&
				!constraints.is_constrained(newLoc, node->level + 1) &&
				!constraints.is_constrained(node->location * solver.map_size + newLoc, node->level + 1)) // valid move
			{
				std::list<MDDNode*>::reverse_iterator child = closed.rbegin();
				bool find = false;
				for (; child != closed.rend() && ((*child)->level == node->level + 1); ++child)
					if ((*child)->location == newLoc)  // If the child node exists
					{
						if ((*child)->heading == -1) { //if no heading info
							(*child)->parents.push_back(node); // then add corresponding parent link and child link
							find = true;
							break;
						}
						else if ((*child)->location == solver.goal_location) { //if goal location ignore heading
							(*child)->parents.push_back(node); // then add corresponding parent link and child link
							find = true;
							break;
						}
						else if ((*child)->heading == new_heading) {//child heading equal to node heading
							(*child)->parents.push_back(node); // then add corresponding parent link and child link
							find = true;
							break;
						}

						
					}
				if (!find) // Else generate a new mdd node
				{
					MDDNode* childNode = new MDDNode(newLoc, node);
					childNode->heading = new_heading;
					childNode->row = newLoc / solver.num_col;
					childNode->col = newLoc % solver.num_col;
                    childNode->position_fraction = next_position_fraction;


                    open.push(childNode);
					closed.push_back(childNode);
				}
			}
		}
	}
	// Backward
	for (int t = numOfLevels - 1; t > 0; t--)
	{
		for (std::list<MDDNode*>::iterator it = levels[t].begin(); it != levels[t].end(); ++it)
		{
			for (std::list<MDDNode*>::iterator parent = (*it)->parents.begin(); parent != (*it)->parents.end(); parent++)
			{
				if ((*parent)->children.empty()) // a new node
				{
					levels[t - 1].push_back(*parent);
				}
				(*parent)->children.push_back(*it); // add forward edge	
			}
		}
	}

	// Delete useless nodes (nodes who don't have any children)
	for (std::list<MDDNode*>::iterator it = closed.begin(); it != closed.end(); ++it)
		if ((*it)->children.empty() && (*it)->level < numOfLevels - 1)
			delete (*it);
	return true;
}

//Build mdd with given heading and start info.
template<class Map>
bool MDD<Map>::buildMDD( ConstraintTable& constraints,
	int numOfLevels, SingleAgentICBS<Map> & solver,int start,int start_time, int start_heading)
{
	MDDNode* root = new MDDNode(-1, NULL); // Root
	root->heading = start_heading;
	root->row = solver.start_location / solver.num_col;
	root->col = solver.start_location % solver.num_col;
	root->position_fraction = solver.al->agents[solver.agent_id]->position_fraction;
	root->malfunction_left = solver.al->agents[solver.agent_id]->malfunction_left;
	root->next_malfunction = solver.al->agents[solver.agent_id]->next_malfuntion;
	std::queue<MDDNode*> open;
	std::list<MDDNode*> closed;
	open.push(root);
	closed.push_back(root);
	levels.resize(numOfLevels);
	//cout << "start: " << solver.start_heading << endl;
	//cout << "goal: " << solver.goal_location << endl;

	while (!open.empty())
	{
		MDDNode* node = open.front();
		open.pop();
		// Here we suppose all edge cost equals 1
		if (node->level == numOfLevels - 1)
		{
			levels[numOfLevels - 1].push_back(node);
			if(!open.empty())
			{
				//while (!open.empty())
				//{
				//	MDDNode* node = open.front();
				//	open.pop();
				//	cout << "loc: " << node->location << " heading: " << node->heading<<" h "<< solver.my_heuristic[node->location].heading[node->heading] <<" "<< solver.my_heuristic[node->location].heading.count(node->heading)<< endl;
				//	
				//}
				
				std::cerr << "Failed to build MDD!" << std::endl;
				exit(1);
			}
			break;
		}
		// We want (g + 1)+h <= f = numOfLevels - 1, so h <= numOfLevels - g. -1 because it's the bound of the children.
		double heuristicBound = numOfLevels - node->level - 2+ 0.001; 

		vector<Transition> transitions;
		if (node->malfunction_left > 0) {
			Transition move;
			move.first = node->location;
			move.second = 4;
			move.position_fraction = node->position_fraction;
			transitions.push_back(move);
		}
        else if(node->location == -1){
            Transition move;
            move.first = -1;
            move.second = 4;
            move.position_fraction = node->position_fraction;
            transitions.push_back(move);

            Transition move2;
            move2.first = start;
            move2.second = 4;
            move2.position_fraction = node->position_fraction;
            transitions.push_back(move2);

        }
		else if (node->position_fraction + solver.al->agents[solver.agent_id]->speed >= 0.97) {
			if (node->position_fraction == 0)
				transitions = solver.ml->get_transitions(node->location, node->heading, false);
			else
				transitions = solver.ml->get_transitions(node->location, node->heading, true);
		}
		else if (node->position_fraction == 0) {
			Transition move;
			move.first = node->location;
			move.second = 4;
			move.position_fraction = node->position_fraction;

			transitions.push_back(move);

			Transition move2;
			move2.first = node->location;
			move2.second = 4;
			move2.position_fraction = node->position_fraction + solver.al->agents[solver.agent_id]->speed;

			transitions.push_back(move2);
		}
		else { //<0.97 and po_frac not 0


			Transition move2;
			move2.first = node->location;
			move2.second = 4;
			move2.position_fraction = node->position_fraction + solver.al->agents[solver.agent_id]->speed;

			transitions.push_back(move2);


		}
		//cout << "current " << node->location << " heading " << node->heading << endl;
		for (const auto move : transitions)
		{
			int next_malfunction_left = node->malfunction_left > 0 ? node->malfunction_left - 1 : node->malfunction_left;
			int next_next_malfuntion = (node->next_malfunction > 0 && node->malfunction_left == 0) ? node->next_malfunction - 1 : node->next_malfunction;
			if (node->parent != NULL && next_next_malfuntion == 0 && node->next_malfunction == 1) {
				next_malfunction_left = solver.max_malfunction;
			}

			float next_position_fraction = move.position_fraction;

			int new_heading;
			if (node->heading == -1) //heading == -1 means no heading info
				new_heading = -1;
			else
				if (move.second == 4) //move == 4 means wait
					new_heading = node->heading;
				else
					new_heading = move.second;
			int newLoc = move.first;
			//cout << "newLoc " << newLoc << " heading " << new_heading<<" h "<< solver.my_heuristic[newLoc].heading[new_heading] << endl;

			if (solver.my_heuristic[newLoc].heading.count(new_heading) && solver.my_heuristic[newLoc].heading[new_heading] < heuristicBound &&
				!constraints.is_constrained(newLoc, start_time+node->level + 1) &&
				!constraints.is_constrained(node->location * solver.map_size + newLoc, start_time+node->level + 1)) // valid move
			{
				std::list<MDDNode*>::reverse_iterator child = closed.rbegin();
				bool find = false;
				for (; child != closed.rend() && ((*child)->level == node->level + 1); ++child)
					if ((*child)->location == newLoc)  // If the child node exists
					{
						if ((*child)->heading == -1) { //if no heading info
							(*child)->parents.push_back(node); // then add corresponding parent link and child link
							find = true;
							break;
						}
						else if ((*child)->location == solver.goal_location) { //if goal location ignore heading
							(*child)->parents.push_back(node); // then add corresponding parent link and child link
							find = true;
							break;
						}
						else if ((*child)->heading == new_heading) {//child heading equal to node heading
							(*child)->parents.push_back(node); // then add corresponding parent link and child link
							find = true;
							break;
						}

						
					}
				if (!find) // Else generate a new mdd node
				{
					MDDNode* childNode = new MDDNode(newLoc, node);
					childNode->heading = new_heading;
					childNode->row = newLoc / solver.num_col;
					childNode->col = newLoc % solver.num_col;
					childNode->malfunction_left = next_malfunction_left;
					childNode->next_malfunction = next_next_malfuntion;
					childNode->position_fraction = next_position_fraction;
					open.push(childNode);
					closed.push_back(childNode);
				}
			}
		}
	}
	// Backward
	for (int t = numOfLevels - 1; t > 0; t--)
	{
		for (std::list<MDDNode*>::iterator it = levels[t].begin(); it != levels[t].end(); ++it)
		{
			for (std::list<MDDNode*>::iterator parent = (*it)->parents.begin(); parent != (*it)->parents.end(); parent++)
			{
				if ((*parent)->children.empty()) // a new node
				{
					levels[t - 1].push_back(*parent);
				}
				(*parent)->children.push_back(*it); // add forward edge	
			}
		}
	}

	// Delete useless nodes (nodes who don't have any children)
	for (std::list<MDDNode*>::iterator it = closed.begin(); it != closed.end(); ++it)
		if ((*it)->children.empty() && (*it)->level < numOfLevels - 1)
			delete (*it);
	return true;
}

//This build mdd method considers heading information and specify a goal location
template<class Map>
bool MDD<Map>::buildMDD(ConstraintTable& constraints,
	int numOfLevels, SingleAgentICBS<Map> & solver, int start, int start_time, int goal, int start_heading)
{
	MDDNode* root = new MDDNode(start, NULL); // Root
	root->heading = start_heading;
	root->row = solver.start_location / solver.num_col;
	root->col = solver.start_location % solver.num_col;
	std::queue<MDDNode*> open;
	std::list<MDDNode*> closed;
	open.push(root);
	closed.push_back(root);
	levels.resize(numOfLevels);
	//cout << "start: "<< start<<" heading: " << solver.start_heading << endl;
	//cout << "goal: " << goal << endl;
	//cout << "numOfLevels: " << numOfLevels << endl;

	while (!open.empty())
	{
		MDDNode* node = open.front();
		open.pop();
		// Here we suppose all edge cost equals 1
		if (node->level == numOfLevels - 1)
		{
			levels[numOfLevels - 1].push_back(node);
			if (!open.empty())
			{
				while (!open.empty())
				{
					MDDNode* node = open.front();
					open.pop();
					//cout << "loc: " << node->location <<" goal: "<< goal << " heading: " << node->heading<<" level: "<<node->level<<" h "<< getMahattanDistance(node->location, goal, solver.num_col) <<" "<< solver.my_heuristic[node->location].heading.count(node->heading)<< endl;
				}

				std::cerr << "Failed to build MDD!" << std::endl;
				exit(1);
			}
			break;
		}
		// We want (g + 1)+h <= f = numOfLevels - 1, so h <= numOfLevels - g. -1 because it's the bound of the children.
		double heuristicBound = numOfLevels - node->level - 2 + 0.001;

		vector<Transition> transitions;
		if (node->malfunction_left > 1) {
			Transition move;
			move.first = node->location;
			move.second = 4;
			move.position_fraction = node->position_fraction;
			transitions.push_back(move);
		}
		else if (node->position_fraction + solver.al->agents[solver.agent_id]->speed >= 0.97) {
			if (node->position_fraction == 0)
				transitions = solver.ml->get_transitions(node->location, node->heading, false);
			else
				transitions = solver.ml->get_transitions(node->location, node->heading, true);
		}
		else if (node->position_fraction == 0) {
			Transition move;
			move.first = node->location;
			move.second = 4;
			move.position_fraction = node->position_fraction;

			transitions.push_back(move);

			Transition move2;
			move2.first = node->location;
			move2.second = 4;
			move2.position_fraction = node->position_fraction +solver.al->agents[solver.agent_id]->speed;

			transitions.push_back(move2);
		}
		else { //<0.97 and po_frac not 0


			Transition move2;
			move2.first = node->location;
			move2.second = 4;
			move2.position_fraction = node->position_fraction +solver.al->agents[solver.agent_id]->speed;

			transitions.push_back(move2);


		}

		//cout << "nodeent " << node->location << " heading " << node->heading << endl;
		for (const auto move : transitions)
		{
			int next_malfunction_left = node->malfunction_left > 0 ? node->malfunction_left - 1 : node->malfunction_left;
			int next_next_malfuntion = (node->next_malfunction > 0 && node->malfunction_left == 0) ? node->next_malfunction - 1 : node->next_malfunction;
			if (node->parent != NULL && next_next_malfuntion == 0 && node->next_malfunction == 1) {
				next_malfunction_left = solver.max_malfunction;
			}

			float next_position_fraction = move.position_fraction;


			int new_heading;
			if (node->heading == -1) //heading == -1 means no heading info
				new_heading = -1;
			else
				if (move.second == 4) //move == 4 means wait
					new_heading = node->heading;
				else
					new_heading = move.second;
			int newLoc = move.first;
			int next_h_val = getMahattanDistance(newLoc, goal, solver.num_col);
			if (next_h_val < heuristicBound &&
				!constraints.is_constrained(newLoc, start_time+node->level + 1) &&
				!constraints.is_constrained(node->location * solver.map_size + newLoc, start_time + node->level + 1)) // valid move
			{
				std::list<MDDNode*>::reverse_iterator child = closed.rbegin();
				bool find = false;
				for (; child != closed.rend() && ((*child)->level == node->level + 1); ++child)
					if ((*child)->location == newLoc)  // If the child node exists
					{
						if ((*child)->heading == -1) { //if no heading info
							(*child)->parents.push_back(node); // then add corresponding parent link and child link
							find = true;
							break;
						}
						else if ((*child)->location == goal) { //if goal location ignore heading
							(*child)->parents.push_back(node); // then add corresponding parent link and child link
							find = true;
							break;
						}
						else if ((*child)->heading == new_heading) {//child heading equal to node heading
							(*child)->parents.push_back(node); // then add corresponding parent link and child link
							find = true;
							break;
						}


					}
				if (!find) // Else generate a new mdd node
				{
					MDDNode* childNode = new MDDNode(newLoc, node);
					childNode->heading = new_heading;
					childNode->row = newLoc / solver.num_col;
					childNode->col = newLoc % solver.num_col;
					childNode->malfunction_left = next_malfunction_left;
					childNode->next_malfunction = next_next_malfuntion;
					childNode->position_fraction = next_position_fraction;
					
					open.push(childNode);
					closed.push_back(childNode);
				}
			}
		}
	}
	// Backward
	for (int t = numOfLevels - 1; t > 0; t--)
	{
		for (std::list<MDDNode*>::iterator it = levels[t].begin(); it != levels[t].end(); ++it)
		{
			for (std::list<MDDNode*>::iterator parent = (*it)->parents.begin(); parent != (*it)->parents.end(); parent++)
			{
				if ((*parent)->children.empty()) // a new node
				{
					levels[t - 1].push_back(*parent);
				}
				(*parent)->children.push_back(*it); // add forward edge	
			}
		}
	}

	// Delete useless nodes (nodes who don't have any children)
	for (std::list<MDDNode*>::iterator it = closed.begin(); it != closed.end(); ++it)
		if ((*it)->children.empty() && (*it)->level < numOfLevels - 1)
			delete (*it);
	return true;
}

template<class Map>
void MDD<Map>::deleteNode(MDDNode* node)
{
	levels[node->level].remove(node);
	for (std::list<MDDNode*>::iterator child = node->children.begin(); child != node->children.end(); ++child)
	{
		(*child)->parents.remove(node);
		if((*child)->parents.empty())
			deleteNode(*child);
	}
	for (std::list<MDDNode*>::iterator parent = node->parents.begin(); parent != node->parents.end(); ++parent)
	{
		(*parent)->children.remove(node);
		if ((*parent)->children.empty())
			deleteNode(*parent);
	}
}

template<class Map>
void MDD<Map>::clear()
{
	if(levels.empty())
		return;
	for (int i = 0; i < levels.size(); i++)
	{
		for (std::list<MDDNode*>::iterator it = levels[i].begin(); it != levels[i].end(); ++it)
			delete (*it);
	}
}

template<class Map>
MDDNode* MDD<Map>::find(int location, int level)
{
	if(level < levels.size())
		for (std::list<MDDNode*>::iterator it = levels[level].begin(); it != levels[level].end(); ++it)
			if((*it)->location == location)
				return (*it);
	return NULL;
}

template<class Map>
MDD<Map>::MDD(MDD & cpy) // deep copy
{
	levels.resize(cpy.levels.size());
	MDDNode* root = new MDDNode(cpy.levels[0].front()->location, NULL);
	levels[0].push_back(root);
	for(int t = 0; t < levels.size() - 1; t++)
	{
		for (std::list<MDDNode*>::iterator node = levels[t].begin(); node != levels[t].end(); ++node)
		{
			MDDNode* cpyNode = cpy.find((*node)->location, (*node)->level);
			for (std::list<MDDNode*>::const_iterator cpyChild = cpyNode->children.begin(); cpyChild != cpyNode->children.end(); ++cpyChild)
			{
				MDDNode* child = find((*cpyChild)->location, (*cpyChild)->level);
				if (child == NULL)
				{
					child = new MDDNode((*cpyChild)->location, (*node));
					levels[child->level].push_back(child);
					(*node)->children.push_back(child);
				}
				else
				{
					child->parents.push_back(*node);
					(*node)->children.push_back(child);
				}
			}
		}
		
	}
}

template<class Map>
MDD<Map>::~MDD()
{
	clear();
}

template class MDD<MapLoader>;
template class MDD<FlatlandLoader>;


