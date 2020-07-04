#include "ICBSSearch.h"
#include "CorridorReasoning.h"
#include <ctime>
#include <iostream>

#define MAX_K_VERTEX_COVER_EDGES 10

// takes the paths_found_initially and UPDATE all (constrained) paths found for agents from curr to start
// also, do the same for ll_min_f_vals and paths_costs (since its already "on the way").
inline void ICBSSearch::updatePaths(ICBSNode* curr)
{
	for(int i = 0; i < num_of_agents; i++)
		paths[i] = &paths_found_initially[i];
	vector<bool> updated(num_of_agents, false);  // initialized for false

	while (curr->parent != NULL)
	{
		for (list<pair<int, vector<PathEntry>>>::iterator it = curr->paths.begin(); it != curr->paths.end() ; ++it)
		{
			if (!updated[it->first])
			{
				paths[it->first] = &(it->second);
				updated[it->first] = true;
			}
		}
		curr = curr->parent;
	}
}




/*std::vector <std::list< std::pair<int, int> > >* ICBSSearch::collectConstraints(ICBSNode* curr, int agent_id)
{
	std::clock_t t1 = std::clock();
	// extract all constraints on agent_id
	list < tuple<int, int, int> > constraints;
	int max_timestep = -1;
	while (curr != dummy_start) 
	{
		if (curr->agent_id == agent_id) 
		{
			for (auto constraint : curr->constraints)
			{
				constraints.push_back(constraint);
				if (get<2>(constraint) > max_timestep) // calc constraints' max_timestep
					max_timestep = get<2>(constraint);
				if(-1 - get<2>(constraint) > max_timestep) // calc constraints' max_timestep
					max_timestep = -1 - get<2>(constraint);
			}
		}
		curr = curr->parent;
	}


	// initialize a constraint vector of length max_timestep+1. Each entry is an empty list< pair<int,int> > (loc1,loc2)
	vector < list< pair<int, int> > >* cons_vec = new vector < list< pair<int, int> > >(max_timestep + 1, list< pair<int, int> >());
	
	for (list< tuple<int, int, int> >::iterator it = constraints.begin(); it != constraints.end(); it++) 
	{
		if (get<2>(*it) < 0) // time range constraint
		{
			int loc = get<0>(*it);
			int t1 = -1 - get<1>(*it);
			int t2 = -1 - get<2>(*it);			
			for (int i = t1; i <= t2; i++)
				cons_vec->at(i).push_back(make_pair(loc, -1));
		}
		else if (get<0>(*it) < 0) // barrier constraint
		{
			int x1 = (-get<0>(*it) - 1) / num_col, y1 = (-get<0>(*it) - 1) % num_col;
			int x2 = get<1>(*it) / num_col, y2 = get<1>(*it) % num_col;
			if (x1 == x2)
			{
				if (y1 < y2)
					for (int i = 0; i <= std::min(y2 - y1, get<2>(*it)); i++)
						cons_vec->at(get<2>(*it) - i).push_back(make_pair(x1 * num_col + y2 - i, -1));
				else
					for (int i = 0; i <= std::min(y1 - y2, get<2>(*it)); i++)
						cons_vec->at(get<2>(*it) - i).push_back(make_pair(x1 * num_col + y2 + i, -1));
			}
			else // y1== y2
			{
				if (x1 < x2)
					for (int i = 0; i <= std::min(x2 - x1, get<2>(*it)); i++)
						cons_vec->at(get<2>(*it) - i).push_back(make_pair((x2 - i) * num_col + y1, -1));
				else
					for (int i = 0; i <= std::min(x1 - x2, get<2>(*it)); i++)
						cons_vec->at(get<2>(*it) - i).push_back(make_pair((x2 + i) * num_col + y1, -1));
			}
		}
		else
			cons_vec->at(get<2>(*it)).push_back(make_pair(get<0>(*it), get<1>(*it)));
	}
	
	runtime_updatecons += std::clock() - t1;
	return cons_vec;
}*/

int ICBSSearch::computeHeuristics(const ICBSNode& curr)
{
	// Conflict graph
	vector<vector<bool>> CG(num_of_agents);
	int num_of_CGnodes = 0, num_of_CGedges = 0;
	for (int i = 0; i < num_of_agents; i++)
		CG[i].resize(num_of_agents, false);
    if (debug_mode)
        cout << "Conflict graph: ";
	for (auto conflict : curr.conflicts)
	{
		if(conflict->p == conflict_priority::CARDINAL && !CG[conflict->a1][conflict->a2])
		{
			CG[conflict->a1][conflict->a2] = true;
			CG[conflict->a2][conflict->a1] = true;
			num_of_CGedges++;
			if (debug_mode)
			    cout << "(" << conflict->a1 << "," << conflict->a2 << "),";
		}
	}
    if (debug_mode)
        cout << endl;

	if (num_of_CGedges < 2)
		return num_of_CGedges;

	// Compute #CG nodes that have edges
	for (int i = 0; i < num_of_agents; i++)
	{
		for (int j = 0; j < num_of_agents; j++)
		{
			if (CG[i][j])
			{
				num_of_CGnodes++;
				break;
			}
		}
	}


	// Minimum Vertex Cover
	if (curr.parent == NULL || // root node of CBS tree or
        num_of_CGedges > MAX_K_VERTEX_COVER_EDGES) // too many edges for k vertex cover method
	{
		return minimumVertexCover(CG);
	}
	if (KVertexCover(CG, num_of_CGnodes, num_of_CGedges, curr.parent->h_val - 1))
		return curr.parent->h_val - 1;
	else if (KVertexCover(CG, num_of_CGnodes, num_of_CGedges, curr.parent->h_val))
		return curr.parent->h_val;
	else
		return curr.parent->h_val + 1;
}

// Find disjoint components and apply k vertex cover or greedy matching on each component
int ICBSSearch::minimumVertexCover(const vector<vector<bool>>& CG)
{
    int rst = 0;
    std::vector<bool> done(num_of_agents, false);
    for (int i = 0; i < num_of_agents; i++)
    {
        if (done[i])
            continue;
        std::vector<int> indices;
        indices.reserve(num_of_agents);
        std::queue<int> Q;
        Q.push(i);
        done[i] = true;
        while (!Q.empty())
        {
            int j = Q.front(); Q.pop();
            indices.push_back(j);
            for (int k = 0; k < num_of_agents; k++)
            {
                if (CG[j][k] && !done[k])
                {
                    Q.push(k);
                    done[k] = true;
                }
            }
        }
        if ((int) indices.size() == 1) //one node -> no edges -> mvc = 0
            continue;
        else if ((int)indices.size() == 2) // two nodes -> only one edge -> mvc = 1
        {
            rst += 1; // add edge weight
            continue;
        }

        std::vector<vector<bool> > subgraph(indices.size(), vector<bool>(indices.size(), false));
        int num_edges = 0;
        for (int j = 0; j < (int) indices.size(); j++)
        {
            for (int k = j + 1; k < (int)indices.size(); k++)
            {
                subgraph[j][k] = CG[indices[j]][indices[k]];
                subgraph[k][j] = CG[indices[k]][indices[j]];
                if (subgraph[j][k])
                    num_edges++;
            }
        }

        if (num_edges < MAX_K_VERTEX_COVER_EDGES)
        {
            for (int i = 1; i < (int)indices.size(); i++)
            {
                if (KVertexCover(subgraph, (int)indices.size(), num_edges, i))
                {
                    rst += i;
                    break;
                }
                runtime = (std::clock() - start);
                if (runtime > time_limit)
                    return -1; // run out of time
            }
        }
        else
        {
            return greedyMatching(subgraph);
        }
    }
    return rst;
}

int ICBSSearch::greedyMatching(const vector<vector<bool>>& CG) const
{
    int rst = 0;
    std::vector<bool> selected(CG.size(), false);
    for (int i = 0; i < (int)CG.size(); i++)
    {
        if (selected[i])
            continue;
        for (int j = i + 1; j < (int)CG.size(); j++)
        {
            if(CG[i][j] > 0 && !selected[j])
            {
                rst += 1; // select the edge between i and j
                selected[i] = true;
                selected[j] = true;
            }
        }
    }
    return rst;
}

// Whether there exists a k-vertex cover solution
bool ICBSSearch::KVertexCover(const vector<vector<bool>>& CG, int num_of_CGnodes, int num_of_CGedges, int k)
{
	if (num_of_CGedges == 0)
		return true;
	else if (num_of_CGedges > k * num_of_CGnodes - k) 
		return false;

	int node[2];
	bool flag = true;
	for (int i = 0; i < (int)CG.size() - 1 && flag; i++) // to find an edge
	{
		for (int j = i + 1; j < (int)CG.size() && flag; j++)
		{
			if (CG[i][j])
			{
				node[0] = i;
				node[1] = j;
				flag = false;
			}
		}
	}
	for (int i = 0; i < 2; i++)
	{
		vector<vector<bool>> CG_copy(CG.size());
		CG_copy.assign(CG.cbegin(), CG.cend());
		int num_of_CGedges_copy = num_of_CGedges;
		for (int j = 0; j < (int)CG.size(); j++)
		{
			if (CG_copy[node[i]][j])
			{
				CG_copy[node[i]][j] = false;
				CG_copy[j][node[i]] = false;
				num_of_CGedges_copy--;
			}
		}
		if (KVertexCover(CG_copy, num_of_CGnodes - 1, num_of_CGedges_copy, k - 1))
			return true;
	}
	return false;
}

// deep copy of all conflicts except ones that involve the particular agent
// used for copying conflicts from the parent node to the child nodes
void ICBSSearch::copyConflicts(const std::list<std::shared_ptr<Conflict >>& conflicts,
	std::list<std::shared_ptr<Conflict>>& copy, const list<int>& excluded_agents) const
{
	for (auto conflict : conflicts)
	{
		bool found = false;
		for (auto a : excluded_agents)
		{
			if (conflict->a1 == a || conflict->a2 == a)
			{
				found = true;
				break;
			}
		}
		if (!found)
		{
			copy.push_back(conflict);
		}
	}
}

//void ICBSSearch::copyConflicts(const std::list<std::shared_ptr<CConflict >>& conflicts,
//	std::list<std::shared_ptr<CConflict>>& copy, int excluded_agent) const
//{
//	for (std::list<std::shared_ptr<CConflict>>::const_iterator it = conflicts.begin(); it != conflicts.end(); ++it)
//	{
//		if (get<0>(**it) != excluded_agent && get<1>(**it) != excluded_agent)
//		{
//			copy.push_back(*it);
//		}
//	}
//}



void ICBSSearch::findConflicts(ICBSNode& curr)
{

	if (curr.parent != NULL)
	{
		if (debug_mode)
			cout << "copy from parent" << endl;
		// Copy from parent、
		list<int> new_agents;
		for (auto p : curr.paths)
		{
			new_agents.push_back(p.first);
		}
		copyConflicts(curr.parent->conflicts, curr.conflicts, new_agents);
		copyConflicts(curr.parent->unknownConf, curr.unknownConf, new_agents);

		if (debug_mode) {
            cout << "Find Conflicts" << endl;
            cout << "Number of existing unknown: "<<curr.unknownConf.size()<<endl;
        }
		// detect new conflicts
		for (list<int>::iterator it = new_agents.begin(); it != new_agents.end(); ++it)
		{
			int a1 = *it;
			//collect conflict from path;
			for (size_t t = 0; t < paths[a1]->size(); t++) {
				if (paths[a1]->at(t).conflist != NULL && paths[a1]->at(t).conflist->size() != 0) {
					int preciousConflit[4];
					for (auto& con : *(paths[a1]->at(t).conflist)) {
						if (debug_mode)
							cout << "l<" << get<0>(*con) << "," << get<1>(*con) << ","
							<< "(" << get<2>(*con) / num_col << "," << get<2>(*con) % num_col << ")" << ","
							<< "(" << get<3>(*con) / num_col << "," << get<3>(*con) % num_col << ")" << ","
							<< get<4>(*con) << "," << get<5>(*con) << ">; ";
						//cout << "(" << abs(preciousConflit[2] - get<2>(*con)) << ")";
//						if (preciousConflit[0] == get<0>(*con) &&
//							preciousConflit[1] == get<1>(*con) &&
//							(abs(preciousConflit[2] - get<2>(*con)) == num_col || abs(preciousConflit[2] - get<2>(*con)) == 1 || preciousConflit[2] == get<2>(*con)) &&
//							abs(get<4>(*con) - preciousConflit[3]) == 1
//							) {
//							preciousConflit[0] = get<0>(*con);
//							preciousConflit[1] = get<1>(*con);
//							preciousConflit[2] = get<2>(*con);
//							preciousConflit[3] = get<4>(*con);
//							//cout << "continues conf, jump" << endl;
//							continue;
//						}
						std::shared_ptr<Conflict> newConf(new Conflict());

						if (targetReasoning && (get<3>(*con) < 0) && (get<4>(*con) >= paths[get<0>(*con)]->size() - 1)) {
							newConf->targetConflict(get<0>(*con), get<1>(*con), get<2>(*con), get<4>(*con)+ get<5>(*con), kDelay);
						}
						//else if (targetReasoning && (get<3>(*con) < 0) && (get<4>(*con) >= paths[get<1>(*con)]->size() - 1)) {
						//	newConf->targetConflict(get<1>(*con), get<0>(*con), get<2>(*con), get<4>(*con)+ get<5>(*con), kDelay);
						//}
						else if (get<3>(*con) < 0) {
							if(get<4>(*con) >= paths[get<0>(*con)]->size() - 1)
								newConf->vertexConflict(get<0>(*con), get<1>(*con), get<2>(*con), get<4>(*con) + get<5>(*con),0, kDelay);
							else
								newConf->vertexConflict(get<0>(*con), get<1>(*con), get<2>(*con), get<4>(*con), get<5>(*con), kDelay);
						}
						else {
							newConf->edgeConflict(get<0>(*con), get<1>(*con), get<2>(*con), get<3>(*con), get<4>(*con));
						}

						curr.unknownConf.emplace_back(newConf);
//						preciousConflit[0] = get<0>(*con);
//						preciousConflit[1] = get<1>(*con);
//						preciousConflit[2] = get<2>(*con);
//						preciousConflit[3] = get<4>(*con);

					}
					delete paths[a1]->at(t).conflist;
				}
			}

			if (ignoreFinishedAgent)
				continue;

			for (int a2 = 0; a2 < num_of_agents; a2++)
			{//low level search can't find target conflict if a1<a2
				if (a1 == a2)
					continue;
				if (paths[a1]->size() == 0 || paths[a2]->size() == 0)
					continue;
				bool skip = false;
				for (list<int>::iterator it2 = new_agents.begin(); it2 != it; ++it2)
				{
					if (*it2 == a2)
					{
						skip = true;
						break;
					}
				}
				findTargetConflicts(a1, a2, curr);

			}
		}
	}
	else
	{
		if (debug_mode)
			cout << "Find Conflicts" << endl;
		for (int a1 = 0; a1 < num_of_agents; a1++)
		{

			//collect conflicts from path
			for (size_t t = 0; t < paths[a1]->size(); t++) {

				if (paths[a1]->at(t).conflist == NULL || paths[a1]->at(t).conflist->size() == 0)
					continue;

				int preciousConflit[4];
				for (auto& con : *(paths[a1]->at(t).conflist)) {
//					if (preciousConflit[0]== get<0>(*con) &&
//						preciousConflit[1] == get<1>(*con)&&
//						(abs(preciousConflit[2] - get<2>(*con)) ==num_col|| abs(preciousConflit[2] - get<2>(*con)) == 1|| preciousConflit[2] == get<2>(*con))&&
//						abs(get<4>(*con)- preciousConflit[3]) == 1
//						) {
//
//						preciousConflit[0] = get<0>(*con);
//						preciousConflit[1] = get<1>(*con);
//						preciousConflit[2] = get<2>(*con);
//						preciousConflit[3] = get<4>(*con);
//						//cout << "continues conf, jump" << endl;
//
//						continue;
//					}

					std::shared_ptr<Conflict> newConf(new Conflict());
					
					if (targetReasoning && (get<3>(*con) < 0) && (get<4>(*con) > paths[get<0>(*con)]->size() - 1)) {
						newConf->targetConflict(get<0>(*con), get<1>(*con), get<2>(*con), get<4>(*con), kDelay);
					}
					//else if (targetReasoning && (get<3>(*con) < 0) && (get<4>(*con) > paths[get<1>(*con)]->size() - 1)) {
					//	newConf->targetConflict(get<1>(*con), get<0>(*con), get<2>(*con), get<4>(*con), kDelay);
					//}
					else if (get<3>(*con) < 0) {
						if (get<4>(*con) >= paths[get<0>(*con)]->size() - 1)
							newConf->vertexConflict(get<0>(*con), get<1>(*con), get<2>(*con), get<4>(*con) + get<5>(*con), 0, kDelay);
						else
							newConf->vertexConflict(get<0>(*con), get<1>(*con), get<2>(*con), get<4>(*con), get<5>(*con), kDelay);
					}
					else {
						newConf->edgeConflict(get<0>(*con), get<1>(*con), get<2>(*con), get<3>(*con), get<4>(*con));
					}
					if (debug_mode)
					cout << "<" << get<0>(*con) << "," << get<1>(*con) << ","
						<< "(" << get<2>(*con) / num_col << "," << get<2>(*con) % num_col << ")" << ","
						<< "(" << get<3>(*con) / num_col << "," << get<3>(*con) % num_col << ")" << ","
						<< get<4>(*con) << "," << get<5>(*con) << ">; ";
					curr.unknownConf.emplace_back(newConf);
//					preciousConflit[0] = get<0>(*con);
//					preciousConflit[1] = get<1>(*con);
//					preciousConflit[2] = get<2>(*con);
//					preciousConflit[3] = get<4>(*con);



				}
				delete paths[a1]->at(t).conflist;
			}

			if (ignoreFinishedAgent)
				continue;
			for (int a2 = 0; a2 < num_of_agents; a2++)
			{//low level search can't find target conflict if a1<a2
				if (a1 == a2)
					continue;
				if (paths[a1]->size() == 0 || paths[a2]->size() == 0)
					continue;

				findTargetConflicts(a1, a2, curr);
				
			}
		}
	}
	if (debug_mode)
		cout  << endl;
}

void ICBSSearch::findTargetConflicts(int a1, int a2, ICBSNode& curr) {
	size_t min_path_length = paths[a1]->size() < paths[a2]->size() ? paths[a1]->size() : paths[a2]->size();
	//collect conflict from path;
	if (a1 == a2) {
		return;
	}


	if (paths[a1]->size() < paths[a2]->size())
	{
		//short one a1_ longer one a2_
		//current short after goal code should work good for k robust
		int a1_ = a1;
		int a2_ = a2;
		int loc1 = paths[a1_]->back().location;// short one's goal location
		for (size_t timestep = min_path_length; timestep < paths[a2_]->size(); timestep++)
		{
			//for (int k = 0; k <= kDelay; k++) {
			//	//in future longer one can't pass through the goal location of shorter one.
			//	if (timestep + k >= paths[a2_]->size())
			//		continue;
				int loc2 = paths[a2_]->at(timestep).location;
				if (loc1 == loc2)
				{
					std::shared_ptr<Conflict> conflict(new Conflict());
					if (targetReasoning)
					{
						conflict->targetConflict(a1_, a2_, loc1, timestep, kDelay);
					}
					else
					{
						conflict->vertexConflict(a1_, a2_, loc1, timestep,0,kDelay);
					}
					if (debug_mode)
						cout << "<" << a1_ << "," << a2_ << ","
						<< "(" << loc1 / num_col << "," << loc1 % num_col << ")" << ","
						<< "(,)" << ","
						<< timestep << "," "0" << ">; ";
					curr.unknownConf.push_back(conflict);
				}
			//}
			
		}
	}
}

//void ICBSSearch::deleteRectConflict(ICBSNode& curr, const Conflict& conflict)
//{
//	list<shared_ptr<Conflict>>::iterator it;
//	if (!curr.rectCardinalConf.empty())
//	{
//		for (it = curr.rectCardinalConf.begin(); it != curr.rectCardinalConf.end(); ++it)
//		{
//			if ((**it) == conflict)
//			{
//				curr.rectCardinalConf.erase(it);
//				return;
//			}
//		}
//	}
//	if (!curr.rectSemiConf.empty())
//	{
//		for (it = curr.rectSemiConf.begin(); it != curr.rectSemiConf.end(); ++it)
//		{
//			if ((**it) == conflict)
//			{
//				curr.rectSemiConf.erase(it);
//				return;
//			}
//		}
//	}
//	if (!curr.rectNonConf.empty())
//	{
//		for (it = curr.rectNonConf.begin(); it != curr.rectNonConf.end(); ++it)
//		{
//			if ((**it) == conflict)
//			{
//				curr.rectNonConf.erase(it);
//				return;
//			}
//		}
//	}
//}




template<class Map>
bool MultiMapICBSSearch<Map>::isCorridorConflict(std::shared_ptr<Conflict>& corridor, const std::shared_ptr<Conflict>& con, bool cardinal, ICBSNode* node)
{
    if(debug_mode){
        cout<<"Check is corridor conflict"<<endl;
    }
	CorridorReasoning<Map> cp;
	int a[2] = {con->a1, con->a2};
	int  loc1, loc2, timestep;
	constraint_type type;
	int kConflict = con->k;
	std::tie(loc1, loc2, timestep, type) = con->constraint1.back();
	if (kDelay > 0)
		timestep = con->t;
    if(loc1 == search_engines[con->a1]->start_location || loc1 == search_engines[con->a2]->start_location){
        return false;
    }
	//cout << "loc1 " << con->originalConf1<< " loc2 "<< con->originalConf2 << " timestep " << con->t << " k " << kConflict << endl;
	//cout << "loc1 " << loc1 << " loc2 " << loc2 << " timestep " << timestep << " k " << kConflict << endl;

	int curr = -1;
	if (ml->getDegree(loc1) == 2)
	{
		curr = loc1;
		if (loc2 >= 0)
			timestep--;
	}
	else if (ml->getDegree(loc2) == 2)
		curr = loc2;
	if (curr <= 0) {
        if(debug_mode){
            cout<<"degree not 2"<<endl;
        }
        return false;
    }

	int t[2];
	t[0] = cp.getEnteringTime(*paths[a[0]], *paths[a[1-0]], timestep, ml);
	t[1] = cp.getEnteringTime(*paths[a[1]], *paths[a[1 - 1]], timestep+kConflict, ml);

	if (t[0] > t[1])
	{
		int temp = t[0]; t[0] = t[1]; t[1] = temp;
		temp = a[0]; a[0] = a[1]; a[1] = temp;
	}

	bool chasing = false;
	int u[2];//get entering location
	int inerU[2];//get first location inside corridor

    for (int i = 0; i < 2; i++) {
        if(t[i]+1 >=paths[a[i]]->size())
            return false;
        u[i] = paths[a[i]]->at(t[i]).location;
        inerU[i] = paths[a[i]]->at(t[i]+1).location;
    }
	if (u[0] == u[1]) {
		chasing = true;
	}
	if (!chasing) {
		for (int i = 0; i < 2; i++)//check does one entering location lead to another's entering location
		{
			bool found = false;
			for (int time = t[i]; time < paths[a[i]]->size() && !found; time++)
			{
				if (paths[a[i]]->at(time).location == u[1 - i])
					found = true;
			}
			if (!found)
				return false;
		}
	}


	std::pair<int, int> edge; // one edge in the corridor
	int k = getCorridorLength(*paths[a[0]], t[0], u[1], edge);

	if (chasing) {
	    if (debug_mode)
	        cout<<"chasing"<<endl;

//        if (al.agents[a[0]]->speed == al.agents[a[1]]->speed)
//            return false;

	    //get exit location
        int e[2];
        e[0] = cp.getExitTime(*paths[a[0]], *paths[a[1 - 0]], t[0] + 1, ml);
        e[1] = cp.getExitTime(*paths[a[1]], *paths[a[1 - 1]], t[1] + 1, ml);
        int el[2];
        el[0] = paths[a[0]]->at(e[0]).location;
        el[1] = paths[a[1]]->at(e[1]).location;
        if( !((e[0]>e[1] && t[0]<t[1]) || (e[0]<e[1] && t[0]>t[1]))){
            if(debug_mode)
                cout<<"not crossing"<<endl;
            return false;
        }

        if(el[0]==el[1]) {

            std::pair<int, int> edge_empty = make_pair(-2, -2);
            updateConstraintTable(node, a[0]);
            //get exit time for a1
            int a1exit = cp.getBypassLength(search_engines[a[0]]->start_location, el[0], search_engines[a[0]]->start_heading,(*paths[a[0]])[e[0]].heading, edge_empty, ml, num_col, map_size, constraintTable, MAX_COST,
                                        paths[a[0]]->front(), al.agents[a[0]]->speed);
            int a1exit_ = cp.getBypassLength(search_engines[a[0]]->start_location, el[0],search_engines[a[0]]->start_heading,(*paths[a[0]])[e[0]].heading, edge, ml, num_col, map_size, constraintTable, a1exit,
                                            paths[a[0]]->front(), al.agents[a[0]]->speed);
            int a1entrance = cp.getBypassLength(search_engines[a[0]]->start_location, inerU[0],search_engines[a[0]]->start_heading,(*paths[a[0]])[t[0]+1].heading, edge_empty, ml, num_col, map_size, constraintTable, MAX_COST,
                                        paths[a[0]]->front(), al.agents[a[0]]->speed);

            updateConstraintTable(node, a[1]);
            //get exit time for a2
            int a2exit = cp.getBypassLength(search_engines[a[1]]->start_location, el[1],search_engines[a[1]]->start_heading,(*paths[a[1]])[e[1]].heading, edge_empty, ml, num_col, map_size, constraintTable, MAX_COST,
                                        paths[a[1]]->front(), al.agents[a[1]]->speed);
            int a2exit_ = cp.getBypassLength(search_engines[a[1]]->start_location,el[1],search_engines[a[1]]->start_heading,(*paths[a[1]])[e[1]].heading, edge, ml, num_col, map_size, constraintTable, a2exit,
                                            paths[a[1]]->front(), al.agents[a[1]]->speed);
            int a2entrance = cp.getBypassLength(search_engines[a[1]]->start_location,inerU[1],search_engines[a[1]]->start_heading,(*paths[a[1]])[t[1]+1].heading, edge_empty, ml, num_col, map_size, constraintTable, MAX_COST,
                                        paths[a[1]]->front(), al.agents[a[1]]->speed);
            int earlyAgent;
            int early_exit;
            int early_entrance;
            int lateAgent;
            int late_entrance;
            int late_exit;
            if (a1exit <= a2exit) {
                earlyAgent = a[0];
                lateAgent = a[1];
                early_entrance = a2entrance;
                late_entrance = a1entrance;
                late_exit = min(a2exit,a1exit_);
                early_exit = a1exit;
            } else {
                earlyAgent = a[1];
                lateAgent = a[0];
                early_entrance = a1entrance;
                late_entrance = a2entrance;
                late_exit = min(a1exit,a2exit_);
                early_exit = a2exit;
            }
            if(late_entrance>=early_entrance && late_exit>=early_exit){
                corridor = std::shared_ptr<Conflict>(new Conflict());
                corridor->chasingConflict(earlyAgent, lateAgent, loc1, loc2,inerU[0], el[0],
                        late_entrance,late_entrance,early_exit, late_exit, kDelay);

                if (blocked(*(paths[corridor->a1]), corridor->constraint1) && blocked(*(paths[corridor->a2]), corridor->constraint2)) {
                    if (debug_mode)
                        cout<<"blocked"<<endl;
                    return true;
                }
                else {
                    if (debug_mode)
                        cout<<"not blocked"<<endl;
                    return false;
                }
            }
            else{
                if(debug_mode)
                    cout<<"not chasing"<<endl;
                return false;
            }
        }

	}

    if(chasing)
        return false;
	if (trainCorridor1) {
	    if(debug_mode){
	        cout<<"trainCorridor1"<<endl;
	    }

		int e[2];
		e[0] = cp.getExitTime(*paths[a[0]], *paths[a[1 - 0]], t[0] + 1, ml);
		e[1] = cp.getExitTime(*paths[a[1]], *paths[a[1 - 1]], t[1] + 1, ml);


        corridor = std::shared_ptr<Conflict>(new Conflict());
		corridor->trainCorridorConflict(a[0], a[1], inerU[0], inerU[1], t[0]+1, t[1]+1, e[0], e[1], kDelay);
		if (blocked(*(paths[corridor->a1]), corridor->constraint1) && blocked(*(paths[corridor->a2]), corridor->constraint2)) {
		    if (debug_mode)
		    cout<<"blocked"<<endl;
            return true;
        }
        else {
            if (debug_mode)
            cout<<"not blocked"<<endl;
            return false;
        }
	}
	if (corridor2)
	{
        if(debug_mode){
            cout<<"corridor2"<<endl;
        }
        int e[2];
        e[0] = cp.getExitTime(*paths[a[0]], *paths[a[1]], t[0] + 1, ml);
        e[1] = cp.getExitTime(*paths[a[1]], *paths[a[0]], t[1] + 1, ml);

	    if(debug_mode){
	        cout  <<"Agents:" << a[0] <<"," <<a[1] <<endl;
            cout << t[0] << "," << t[1] << ",(" << u[0]/num_col<<","<<u[0]%num_col << ")," << "(" << u[1]/num_col<<","<<u[1]%num_col << "),"<< k << endl;
            cout << e[0] << "," << e[1] << ""<<endl;
        }

		std::pair<int, int> edge_empty = make_pair(-2, -2);
		updateConstraintTable(node, a[0]);
		int t3 = cp.getBypassLength(search_engines[a[0]]->start_location, u[1], search_engines[a[0]]->start_heading,(*paths[a[0]])[e[0]].heading, edge_empty, ml, num_col, map_size, constraintTable, MAX_COST,paths[a[0]]->front(),al.agents[a[0]]->speed);
        int t3_ = cp.getBypassLength(search_engines[a[0]]->start_location, u[1], search_engines[a[0]]->start_heading,(*paths[a[0]])[e[0]].heading, edge, ml, num_col, map_size, constraintTable, t3 + 2 * k + 1, paths[a[0]]->front(), al.agents[a[0]]->speed);

		updateConstraintTable(node, a[1]);
		int t4 = cp.getBypassLength(search_engines[a[1]]->start_location, u[0],search_engines[a[1]]->start_heading,(*paths[a[1]])[e[1]].heading, edge_empty, ml, num_col, map_size, constraintTable, MAX_COST, paths[a[1]]->front(), al.agents[a[1]]->speed);
		int t4_ = cp.getBypassLength(search_engines[a[1]]->start_location, u[0],search_engines[a[1]]->start_heading,(*paths[a[1]])[e[1]].heading, edge, ml, num_col, map_size, constraintTable, t4 + 2 * k + 1, paths[a[1]]->front(), al.agents[a[1]]->speed);
		assert(t3<MAX_COST);
        assert(t4<MAX_COST);


        if (abs(t3 - t4) <= k+kDelay && t3_ >= t3 && t4_ >= t4)
		{

			corridor = std::shared_ptr<Conflict>(new Conflict());
			corridor->corridorConflict(a[0], a[1], u[1], u[0], t3, t4, t3_, t4_, k,kDelay);
			bool block=true;

			if (corridor->constraint1.size() != 0 && !blocked(*(paths[corridor->a1]), corridor->constraint1)) {
				block = false;
			}
			if (corridor->constraint2.size() != 0 && !blocked(*(paths[corridor->a2]), corridor->constraint2)) {
				block = false;
			}

			if (block) {
                if(debug_mode){
                    cout<<"is corridor"<<endl;
                }
				return true;
			}
            if(debug_mode){
                cout<<"not blocked"<<endl;
            }

		}

	}
	
	if (!corridor4)
		return false;
	else if (cardinalCorridorReasoning && !cardinal)
		return false;
	// try 4-way splitting then
	if (t[1] > t[0] + 2 * k - 1)
		return false;
	//cout << "iscorridor7" << endl;

	//get h
	std::tuple<int, int, int> key;
	if (u[0] < u[1])
	{
		key = std::make_tuple(u[0], u[1], k);
	}
	else
	{
		key = std::make_tuple(u[1], u[0], k);
	}
	
	CorridorTable::const_iterator got = corridorTable.find(key);
	int h;
	if (got == corridorTable.end())
	{
		h = cp.getBypassLength(u[0], u[1], edge, ml, num_col, map_size);
		corridorTable[key] = h;
	}
	else
	{
		h = got->second;
	}
	if (h > 0 && k >= h - 1)
		return false;
	//cout << "iscorridor8" << endl;

	for (int i = 0; i < 2; i++)
	{
		bool satisfied = false;
		for (int j = t[0] + k; j <= t[0] + std::min(2 * k, h - 1); j++)
		{
			if (getLocation(*paths[a[i]], j) == u[1-i])
			{
				satisfied = true;
				break;
			}
		}
		if (!satisfied)
			return false;
	}
	//cout << "iscorridor9" << endl;

	corridor = std::shared_ptr<Conflict>(new Conflict());
	corridor->corridorConflict(a[0], a[1], u[0], u[1], t[0], t[1], k, h,kDelay);
	// TODO: test whether block all paths
	return true;
}



void ICBSSearch::removeLowPriorityConflicts(std::list<std::shared_ptr<Conflict>>& conflicts) const
{
	if (conflicts.empty())
		return;
	std::unordered_map<int, std::shared_ptr<Conflict> > keep;
	std::list<std::shared_ptr<Conflict>> to_delete;
	for (auto conflict : conflicts)
	{
		int a1 = conflict->a1, a2 = conflict->a2;
		int key = a1 * num_of_agents + a2;
		auto p = keep.find(key);
		if (p == keep.end())
		{
			keep[key] = conflict;
		}
		else if (*(p->second) < *conflict)
		{
			to_delete.push_back(p->second);
			keep[key] = conflict;
		}
		else
		{
			to_delete.push_back(conflict);
		}
	}

	for (auto conflict : to_delete)
	{
		conflicts.remove(conflict);
	}
}


bool ICBSSearch::traverse(const Path& path, int loc, int t) const
{
	if (t >= path.size())
		if (loc == path.back().location)
			return true;
		else
			return false;
	else if (t >= 0 && path[t].location == loc) {
		//cout << "t: " << t << " loc: " << loc << " path: " << path[t].location << endl;
		return true;

	}
	else 
		return false;
}


bool ICBSSearch::blocked(const Path& path, const std::list<Constraint>& constraints) const
{
	for (auto constraint : constraints)
	{
		int x, y, t;
		constraint_type type;
		tie(x, y, t, type) = constraint;
		//cout << x << "," << y << "," << t << endl;
		if (type == constraint_type::RANGE) // time range constraint
		{
			for (int i = y; i < t; i++)
			{
				if (traverse(path, x, i))
					return true;
			}
		}
		else if (type == constraint_type::BARRIER) // barrier constraint
		{
			int x1 = x / num_col, y1 = x % num_col;
			int x2 = y / num_col, y2 = y % num_col;
			if (x1 == x2)
			{
				if (y1 < y2)
				{
					for (int i = 0; i <= std::min(y2 - y1, t); i++)
					{
						if (traverse(path, x1 * num_col + y2 - i, t - i))
							return true;
					}
				}
				else
				{
					for (int i = 0; i <= std::min(y1 - y2, t); i++)
					{
						if (traverse(path, x1 * num_col + y2 + i, t - i))
							return true;
					}
				}
			}
			else // y1== y2
			{
				if (x1 < x2)
				{
					for (int i = 0; i <= std::min(x2 - x1, t); i++)
					{
						if (traverse(path, (x2 - i) * num_col + y1, t - i))
							return true;
					}
				}
				else
				{
					for (int i = 0; i <= std::min(x1 - x2, t); i++)
					{
						if (traverse(path, (x2 + i) * num_col + y1, t - i))
							return true;
					}
				}
			}
		}
		else {
			if (traverse(path, x, t))
				return true;
		}
	}
	return false;
}


std::shared_ptr<Conflict> ICBSSearch::chooseConflict(ICBSNode &parent)
{
	if (debug_mode)
		cout << "Start choosing conflict" << endl;

	std::shared_ptr<Conflict> choose;
	if (parent.conflicts.empty() && parent.unknownConf.empty())
		return NULL;
	else if (!parent.conflicts.empty())
	{
		choose = parent.conflicts.back();
		for (auto conflict : parent.conflicts)
		{
			if (*choose < *conflict)
				choose = conflict;
		}
	}
	else
	{
		choose = parent.unknownConf.back();
		for (auto conflict : parent.unknownConf)
		{
			if (conflict->t < choose->t)
				choose = conflict;
		}
	}

	if (debug_mode)
		cout << "Choosing conflict done" << endl;
	return choose;
}


bool ICBSSearch::generateChild(ICBSNode*  node, ICBSNode* curr)
{
	if (debug_mode)
		cout << "generate child" << endl;
	node->parent = curr;
	node->g_val = curr->g_val;
	node->makespan = curr->makespan;
	node->depth = curr->depth + 1;
	node->num_of_dead_agents = curr->num_of_dead_agents;
	std::clock_t t1;

	t1 = std::clock();
	if (std::get<0>(node->constraints.front()) >= 0 &&
		std::get<3>(node->constraints.front()) == constraint_type::LENGTH)
	{
		int x, agent, t;
		constraint_type type;
		tie(x, agent, t, type) = node->constraints.front();
		for (int ag = 0; ag < num_of_agents; ag++)
		{
			if (ag == agent)
			{
				continue;
			}

			if (t > paths[ag]->size()) {
				continue;
			}
			bool replan = false;
			/*for (int k = 0; k <= kDelay; k++) {
				if (t + k < paths[ag]->size() && paths[ag]->at(t + k).location == x) {
					replan = true;
				}
			}*/
			for (int tg = t; tg < paths[ag]->size(); tg++) {
				if (paths[ag]->at(tg).location == x) {
					replan = true;
					break;
				}
			}

			if (replan)
			{
				double lowerbound = (int)paths[ag]->size() - 1;
				if (!findPathForSingleAgent(node, ag, lowerbound))
					return false;
			}
		}
	}
	else
	{
		double lowerbound = (int)paths[node->agent_id]->size() - 1;
		// if (curr->conflict->p == conflict_priority::CARDINAL && curr->conflict->type != conflict_type::CORRIDOR2)
		//	lowerbound += 1;

		if (!findPathForSingleAgent(node, node->agent_id, lowerbound))
			return false;
	}




	

	
	runtime_lowlevel += (std::clock() - t1) * 1000.0 / CLOCKS_PER_SEC;
	
	//Estimate h value
	if (node->parent->g_val == node->g_val)
	{
		node->h_val = node->parent->h_val;
	}
	else if (node->parent->h_val > 1)
	{
		node->h_val = node->parent->h_val - 1;		
	}
	// else if (hasCardinalConflict(*node))
	// {
	// 	node->h_val = 1;
	// }
	else
		node->h_val = 0;
	node->f_val = node->g_val + node->h_val;
    assert(node->num_of_dead_agents > curr->num_of_dead_agents || node->f_val >= curr->f_val);
	t1 = std::clock();
	findConflicts(*node);
	runtime_conflictdetection += (std::clock() - t1)  * 1000.0 / CLOCKS_PER_SEC;

	node->num_of_collisions = node->unknownConf.size() + node->conflicts.size();

	// update handles
	node->open_handle = open_list.push(node);
	HL_num_generated++;
	node->time_generated = HL_num_generated;
	if(debug_mode) {
        cout << "child dead: " <<node->num_of_dead_agents << " makespan: "<<node->makespan << "f: " << node->f_val<< endl;
        cout << "focal_list_threshold: " << get<0>(focal_list_threshold)<<","<< get<1>(focal_list_threshold) <<","<<get<2>(focal_list_threshold) << endl;
    }
	if (make_tuple(node->num_of_dead_agents, node->makespan, node->f_val) <= focal_list_threshold) {
	    if(debug_mode)
	        cout<<"add to focal"<<endl;
        node->focal_handle = focal_list.push(node);
    }
	allNodes_table.push_back(node);


	return true;
}

bool ICBSSearch::hasCardinalConflict(const ICBSNode& node) const
{
	for (auto conflict : node.conflicts)
	{
		if (conflict->p = conflict_priority::CARDINAL)
			return true;
	}
	return false;
}


void ICBSSearch::printPaths() const
{
	for (int i = 0; i < num_of_agents; i++)
	{
		std::cout << "Agent " << i << " (" << paths_found_initially[i].size() - 1 << " -->" <<
			paths[i]->size() - 1 << "): ";
		for (int t = 0; t < paths[i]->size(); t++)
			std::cout << t <<"(" << paths[i]->at(t).location / num_col << "," << paths[i]->at(t).location % num_col << ")->";
		std::cout << std::endl;
	}
}
void ICBSSearch::printPaths(Path& path) const
{

		for (int t = 0; t < path.size(); t++)
			std::cout << "(" << path.at(t).location / num_col << "," << path.at(t).location % num_col << ")->";
		std::cout << std::endl;
}

void ICBSSearch::printBT(const std::string& prefix, const ICBSNode* node, bool isLeft)
{
	if (node != NULL)
	{
		std::cout << prefix;
		std::cout << (isLeft ? "├──" : "└──");
		if (node->conflict != nullptr) {

			// print the value of the node
			std::cout << "<" << node->conflict->a1 << " "
				<< node->conflict->a2 << ",("
				<< node->conflict->originalConf1/num_col<<","
				<<node->conflict->originalConf1 % num_col<<")" << ",("
				<< node->conflict->originalConf2 / num_col << ","
				<< node->conflict->originalConf2 % num_col << ")"
				<< node->conflict->t  <<","<< node->conflict->k<<","<<node->conflict->type<< ">" << std::endl;

			// enter the next tree level - left and right branch

		}
		else {
			std::cout << "No choosen conflict" << std::endl;
		}

		for (int i = 0; i < node->children.size(); i++) {
			printBT(prefix + (isLeft ? "│   " : "    "), node->children[i], i == node->children.size() - 1 ? false: true);
		}
	}
}

void ICBSSearch::printHLTree()
{
	printBT("", dummy_start, false);
}


// adding new nodes to FOCAL (those with min-f-val*f_weight between the old and new LB)
void ICBSSearch::updateFocalList()
{
    ICBSNode* open_head = open_list.top();
    assert(make_tuple(open_head->num_of_dead_agents, open_head->makespan, open_head->f_val) >= min_f_val);
    if (make_tuple(open_head->num_of_dead_agents, open_head->makespan, open_head->f_val) == min_f_val)
        return;
    min_f_val = make_tuple(open_head->num_of_dead_agents, open_head->makespan, open_head->f_val);
    if(debug_mode)
        cout<<"new f min: "<< get<0>(min_f_val)<<","<<get<1>(min_f_val)<<","<<get<2>(min_f_val)<<","<<endl;
    auto new_focal_list_threshold = make_tuple(open_head->num_of_dead_agents, open_head->makespan, (int)(open_head->f_val * focal_w));
	for (ICBSNode* n : open_list) {
	    auto cost = make_tuple(n->num_of_dead_agents, n->makespan, n->f_val);
		if (cost > focal_list_threshold && cost <= new_focal_list_threshold)
			n->focal_handle = focal_list.push(n);
	}
    focal_list_threshold = new_focal_list_threshold;
}

void ICBSSearch::updateReservationTable(bool* res_table, int exclude_agent, const ICBSNode &node) {
	for (int ag = 0; ag < num_of_agents; ag++) 
	{
		if (ag != exclude_agent && paths[ag] != NULL) 
		{
			for (size_t timestep = 0; timestep < node.makespan + 1; timestep++) 
			{
				int id;
				if (timestep >= paths[ag]->size())
					id = paths[ag]->at(paths[ag]->size() - 1).location;
				else// otherwise, return its location for that timestep
					id = paths[ag]->at(timestep).location;
				res_table[timestep * map_size + id] = true;
			}
		}
	}
}


void ICBSSearch::printStrategy() const
{
	switch (cons_strategy)
	{
	case constraint_strategy::CBS:
		cout << "      CBS: ";
		break;
	case constraint_strategy::ICBS:
		cout << "     ICBS: ";
		break;
	case constraint_strategy::CBSH:
		cout << "     CBSH:";
		break;
	case constraint_strategy::CBSH_R:
		cout << "   CBSH-R:";
		break;
	case constraint_strategy::CBSH_CR:
		cout << "  CBSH-CR:";
		break;
	case constraint_strategy::CBSH_RM:
		cout << "  CBSH-RM:";
		break;
	case constraint_strategy::CBSH_GR:
		cout << "  CBSH-GR:";
		break;
	default:
		exit(10);
	}

}


template<class Map>
void MultiMapICBSSearch<Map>::collectConstraints(ICBSNode* curr) {
	allConstraints.clear();
	while (true)
	{
		allConstraints[curr->agent_id].merge(curr->constraints);
		if (curr->parent == NULL)
			break;
		curr = curr->parent;
	}
};


template<class Map>
bool MultiMapICBSSearch<Map>::runICBSSearch()
{
    printStrategy();
    // set timer
    start = std::clock();
    std::clock_t t1;

	runtime_computeh = 0;
	runtime_lowlevel = 0;
	runtime_listoperation = 0;
	runtime_conflictdetection = 0;
	runtime_updatepaths = 0;
	runtime_updatecons = 0;

    initializeDummyStart();

	if (debug_mode)
		cout << "Start searching:" << endl;
	if (screen >= 3)
		al.printCurrentAgentsInitGoal();
	while (!open_list.empty() && !solution_found)
	{
		runtime = (std::clock() - start);
		if (runtime > time_limit)
		{  // timeout
			break;
		}
        t1 = std::clock();
        updateFocalList();
        runtime_listoperation += std::clock() - t1;
		t1 = std::clock();
		ICBSNode* curr = focal_list.top();
		focal_list.pop();
		open_list.erase(curr->open_handle);
		runtime_listoperation += std::clock() - t1;
		// takes the paths_found_initially and UPDATE all constrained paths found for agents from curr to dummy_start (and lower-bounds)
		t1 = std::clock();
		updatePaths(curr);
		runtime_updatepaths += std::clock() - t1;
		if(debug_mode){
            cout << "Child #" << curr->time_generated << " f:" <<curr->f_val <<endl;

        }

		if (cons_strategy == constraint_strategy::CBS)
		{
			t1 = std::clock();
			curr->conflict = chooseConflict(*curr);
			runtime_conflictdetection += std::clock() - t1;
		}
		else if (cons_strategy == constraint_strategy::ICBS) // No heuristics
		{
			t1 = std::clock();
			classifyConflicts(*curr);
			curr->conflict = chooseConflict(*curr);
			runtime_conflictdetection += std::clock() - t1;
		}
		else if(curr->conflict == NULL) //CBSH based, and h value has not been computed yet
		{
			t1 = std::clock();
			classifyConflicts(*curr);
			curr->conflict = chooseConflict(*curr);
			runtime_conflictdetection += std::clock() - t1;

			t1 = std::clock();
			int h = computeHeuristics(*curr);
            curr->h_val = max(curr->h_val, h);
			runtime_computeh += std::clock() - t1;
			curr->f_val = curr->g_val + curr->h_val;

			if (make_tuple(curr->num_of_dead_agents, curr->makespan, curr->f_val) > focal_list_threshold)
			{
				t1 = std::clock();
				curr->open_handle = open_list.push(curr);
				runtime_listoperation += std::clock() - t1;
				continue;
			}

		}



		if (curr->conflict == NULL) //Fail to find a conflict => no conflicts
		{  // found a solution (and finish the while look)
			runtime = (std::clock() - start);
			solution_found = true;
			solution_cost = curr->g_val;
            goal_node = curr;
			collectConstraints(curr);

			if (debug_mode)
				printHLTree();
			if (screen >= 1)
				printPaths();
			cout << solution_cost << " (" << goal_node->num_of_dead_agents << ") ; " << goal_node->makespan << " ; " <<
			    solution_cost - dummy_start->g_val << " ; " <<
				HL_num_expanded << " ; " << HL_num_generated << " ; " <<
				LL_num_expanded << " ; " << LL_num_generated << " ; " << runtime / CLOCKS_PER_SEC << " ; " 
				<< RMTime / CLOCKS_PER_SEC << ";"<<
				num_standard << ";" << num_rectangle << "," <<
				num_corridor2 << ";" << num_chasing << "," << num_target << endl;
			
			break;
		}

		if (debug_mode) {
			cout << "****************" << endl;
			cout << "choose conflict: " << "<" << curr->conflict->a1 << " "
				<< curr->conflict->a2 << ",("
				<< curr->conflict->originalConf1 / num_col << ","
				<< curr->conflict->originalConf1 % num_col << ")" << ",("
				<< curr->conflict->originalConf2 / num_col << ","
				<< curr->conflict->originalConf2 % num_col << ")"
				<< curr->conflict->t << "," << curr->conflict->k << "," << curr->conflict->type << ">" << std::endl;
            cout << "Child #" << curr->time_generated<<endl;


            if (screen>=3)
				printPaths();

		}

		if (debug_mode) {
            cout << "check conflict repeatance" << endl;
            stringstream con;
            //con << *(curr->conflict);
            if (curr->conflict->k == 0) {
                con << min(curr->conflict->a1, curr->conflict->a2) << ",";
                con << max(curr->conflict->a1, curr->conflict->a2);
            } else {
                con << curr->conflict->a1 << ",";
                con << curr->conflict->a2;
            }
            con << ",("
                << curr->conflict->originalConf1 / num_col << ","
                << curr->conflict->originalConf1 % num_col << ")" << ",("
                << curr->conflict->originalConf2 / num_col << ","
                << curr->conflict->originalConf2 % num_col << "),"
                << curr->conflict->t << "," << curr->conflict->k << "," << curr->conflict->type;


            curr->resolvedConflicts.insert(con.str());

            bool stop = false;
            bool noRepeat = true;
            ICBSNode *parent = curr->parent;
            if (parent != NULL) {
                if (debug_mode)
                    cout << "Try find " << con.str() << " in curr's parent nodes" << endl;
                while (!stop) {
                    if (debug_mode)
                        cout << "1";
                    if (parent->parent == NULL) {
                        stop = true;
                        break;
                    }
                    std::unordered_set<std::string>::const_iterator it = parent->resolvedConflicts.find(con.str());
                    if (it != parent->resolvedConflicts.end()) {
                        noRepeat = false;
                        printHLTree();
                    }
                    if (!noRepeat) {
                        cout << "Repeatance detected!!!" << endl;
                        exit(1);
                    }
                    parent = parent->parent;

                }
            } else {
                if (debug_mode)
                    cout << "no parent" << endl;

            }
            if (noRepeat) {
                if (debug_mode)
                    cout << "no repeatance" << endl;
            } else {
                if (debug_mode)
                    cout << "repeatance detected" << endl;
            }
        }



		 //Expand the node
		HL_num_expanded++;
		curr->time_expanded = HL_num_expanded;
		if (curr->conflict->type == conflict_type::RECTANGLE) {

			numOfRectangle += 1;
		}

		vector<ICBSNode*> children;

		if (curr->conflict->type == conflict_type::CORRIDOR4) // 4-way branching
		{
			children.resize(4);
			children[0] = new ICBSNode(curr->conflict->a1);
			children[0]->constraints.push_back(curr->conflict->constraint1.front());
			children[1] = new ICBSNode(curr->conflict->a1);
			children[1]->constraints.push_back(curr->conflict->constraint1.back());
			children[2] = new ICBSNode(curr->conflict->a2);
			children[2]->constraints.push_back(curr->conflict->constraint2.front());
			children[3] = new ICBSNode(curr->conflict->a2);
			children[3]->constraints.push_back(curr->conflict->constraint2.back());
			num_corridor4++;
		}
		//else if (curr->conflict->type == conflict_type::STANDARD && curr->conflict->t == 0 && !option.ignore_t0) {

		//	children.resize(1);
		//	children[0] = new ICBSNode(curr->conflict->a2);
		//	children[0]->constraints = curr->conflict->constraint2;
		//	num_standard++;
		//}
		else // 2-way branching
		{
			
			children.resize(2);
			children[0] = new ICBSNode(curr->conflict->a1);
			children[0]->constraints = curr->conflict->constraint1;
			children[1] = new ICBSNode(curr->conflict->a2);
			children[1]->constraints = curr->conflict->constraint2;
			if (curr->conflict->type == conflict_type::CORRIDOR2)
				num_corridor2++;
			else if (curr->conflict->type == conflict_type::STANDARD) {
                num_standard++;
                if(get<0>(curr->conflict->constraint1.front()) == search_engines[curr->conflict->a1]->start_location
                || get<0>(curr->conflict->constraint2.front()) == search_engines[curr->conflict->a2]->start_location)
                    num_activeConflict++;
            }
            else if (curr->conflict->type == conflict_type::CHASING)
                num_chasing++;
			else if (curr->conflict->type == conflict_type::RECTANGLE) {
				if (curr->conflict->flipType == 1) {
					num_1FlipRectangle++;
				}
				else if (curr->conflict->flipType == 2) {
					num_2FlipRectangle++;
				}
				else {
					num_0FlipRectangle++;
				}
				num_rectangle++;
			}
			else if (curr->conflict->type == conflict_type::TARGET)
				num_target++;
		}

		if (screen >= 2)
		{
			std::cout << "Node " << curr->time_generated << " with f=" << curr->g_val << "+" << curr->h_val
				<< " has " << std::endl;  
			for (auto conflict : curr->conflicts)
				std::cout << *conflict;
			std::cout << "We choose " << *curr->conflict;
		}
		vector<vector<PathEntry>*> copy(paths);
		for (int i = 0; i < children.size(); i++)
		{
			if (children[i]->constraints.size()!=0 && generateChild(children[i], curr))
			{
				curr->children.push_back(children[i]);
				if (screen >= 2)
					std::cout << "Generate #" << children[i]->time_generated
						<< " with cost " << children[i]->g_val
						<< " and " << children[i]->num_of_collisions << " conflicts " << std::endl;

			}
			else
			{
				delete children[i];
			}
			if (i < children.size() - 1)
				paths = copy;
		}
		if (debug_mode) {
			for (int i = 0; i < curr->children.size(); i++)
			{

				cout <<"Child "<<i<< " conflicts:"<< curr->children[i]->unknownConf.size();
	//			for (auto &conit : curr->children[i]->unknownConf) {
	///*				cout << "<" << get<0>(*conit) << "," << get<1>(*conit) << ","
	//					<< "(" << get<2>(*conit) / num_col << "," << get<2>(*conit) % num_col << ")" << ","
	//					<< "(" << get<3>(*conit) / num_col << "," << get<3>(*conit) % num_col << ")" << ","
	//					<< get<4>(*conit) << ">; ";*/


	//			}
				cout << endl;

			}
		}


		curr->clear();
	}  // end of while loop


	if (!solution_found)
	{
        runtime = (std::clock() - start);
	    if (runtime > time_limit)
        {
            cout << "TIMEOUT  ; ";
            solution_cost = -1;
            timeout = true;
        }
	    else
        {
            cout << "No solutions  ; ";
            solution_cost = -2;
            timeout = false;
        }
        updateFocalList();
        cout << solution_cost << " (" << get<0>(min_f_val) << ") ; " << get<1>(min_f_val) << " ; " << get<2>(min_f_val) - dummy_start->g_val << " ; " <<
             HL_num_expanded << " ; " << HL_num_generated << " ; " <<
             LL_num_expanded << " ; " << LL_num_generated << " ; " << runtime / CLOCKS_PER_SEC << " ; "
             << RMTime/CLOCKS_PER_SEC<<";"<<
             num_standard << ";" << num_rectangle << "," <<
             num_corridor2 << ";" << num_chasing << "," << num_target << "," << num_0FlipRectangle << "," <<
             num_1FlipRectangle << "," << num_2FlipRectangle <<
             "|Open|=" << open_list.size() << endl;
        if(debug_mode)
            printHLTree();
	}
	return solution_found;
}


template<class Map>
void MultiMapICBSSearch<Map>::cleanAll() {
	releaseClosedListNodes();
	releaseOpenListNodes();
	search_engines.clear();

	

}


inline void ICBSSearch::releaseClosedListNodes() 
{
	for (list<ICBSNode*>::iterator it = allNodes_table.begin(); it != allNodes_table.end(); it++)
		delete *it;
}

inline void ICBSSearch::releaseOpenListNodes()
{
	while(!open_list.empty())
	{
		ICBSNode* curr = open_list.top();
		open_list.pop();
		delete curr;
	}
}


template<class Map>
MultiMapICBSSearch<Map>::~MultiMapICBSSearch()
{
	for (size_t i = 0; i < search_engines.size(); i++)
		delete (search_engines[i]);
	releaseClosedListNodes();
	if (!mddTable.empty())
	{
		for (int i = 0; i < num_of_agents; i++)
		{
			for (auto mdd : mddTable[i])
			{
				delete mdd.second;
			}
		}
	}
}

template<class Map>
MultiMapICBSSearch<Map>::MultiMapICBSSearch(Map* ml, AgentsLoader* al0, double f_w, constraint_strategy c,
        int time_limit, int screen, int kDlay, options options1): al(*al0)
{
	this->option = options1;
	this->focal_w = f_w;
	this->time_limit = time_limit;
	this->screen = screen;
	if (screen >= 2||options1.debug)
		debug_mode = true;


	if (debug_mode)
		cout << "[CBS] Initializing CBS" << endl;
	cons_strategy = c;
	HL_num_expanded = 0;
	HL_num_generated = 0;

	LL_num_expanded = 0;
	LL_num_generated = 0;
	this->num_col = ml->cols;
	this->ml = ml;
	num_of_agents = al.num_of_agents;
	map_size = ml->rows*ml->cols;
	solution_found = false;
	solution_cost = -1;
	kDelay = kDlay;
	asymmetry_constraint = options1.asymmetry_constraint;
	ignore_t0 = options1.ignore_t0;
	shortBarrier = options1.shortBarrier;
	search_engines = std::vector<SingleAgentICBS<Map>* >(num_of_agents);
	if (debug_mode)
		cout << "Initializing search engines" << endl;

	for (int i = 0; i < num_of_agents; i++) {
// 		if (!al.agents[i].activate)
// 			continue;
		if (debug_mode)
			cout << "initializing agent "<< i << endl;
		int init_loc = ml->linearize_coordinate(al.agents[i]->position.first, al.agents[i]->position.second);
		int goal_loc = ml->linearize_coordinate(al.agents[i]->goal_location.first, al.agents[i]->goal_location.second);
		search_engines[i] = new SingleAgentICBS<Map>(init_loc, goal_loc, ml, al0, i, al.agents[i]->heading, kDelay);
		search_engines[i]->max_malfunction = this->max_malfunction;
		/*if (debug_mode) {
			std::cout << "Heuristic table for " << i << ": ";
			for (int h = 0; h < search_engines[i]->my_heuristic.size(); h++) {
				for (int heading = 0; heading < 5; heading++)
					if (search_engines[i]->my_heuristic[h].heading[heading]<MAX_COST)
					std::cout << "(" << h << ": "<<heading<<": " << search_engines[i]->my_heuristic[h].heading[heading] << ")";
			}
			std::cout << std::endl;

		}*/

	}

	if (debug_mode) {
		cout << "Initializing search engines done" << endl;
	}
}

template<class Map>
void MultiMapICBSSearch<Map>::initializeDummyStart() {
	dummy_start = new ICBSNode();
	dummy_start->agent_id = -1;
    dummy_start->g_val = 0;

	if (debug_mode) {
		cout << "Initializing first solutions" << endl;
		al.printCurrentAgentsInitGoal();
	}
	// initialize paths_found_initially
	paths.resize(num_of_agents, NULL);
	paths_found_initially.resize(num_of_agents);
	ReservationTable res_table(map_size,this->max_malfunction,ignoreFinishedAgent);  // initialized to false
	for (int i = 0; i < num_of_agents; i++) {
		//cout << "******************************" << endl;
		if (screen >= 2) {
			cout << "Find initial path Agent: " << i << endl;
		}
// 		if (!al.agents[i].activate) {
// 			if (debug_mode)
// 				cout << "agent " << i << " not active" << endl;
// 			paths[i] = new vector<PathEntry>;
// 			continue;
// 		}
		// bool found = search_engines[i]->findPath(paths_found_initially[i], focal_w, al.constraintTable,
		//        &res_table, dummy_start->makespan + 1, 0);
        // TODO: for now, I use w=1 for the low-level, because
        //  if the low-level path is suboptimal, mdds, cardinal conflicts and many other parts need to be reconsidered.
        bool found = search_engines[i]->findPath(paths_found_initially[i], 1, al.constraintTable,
                                                 &res_table, dummy_start->makespan + 1, 0);
        LL_num_expanded += search_engines[i]->num_expanded;
        LL_num_generated += search_engines[i]->num_generated;
        paths[i] = &paths_found_initially[i];
        if (found)
        {
            res_table.addPath(i, paths[i]);
            dummy_start->makespan = max(dummy_start->makespan, (int)paths_found_initially[i].size() - 1);
            dummy_start->g_val += paths[i]->size() - 1;
        }
		else
            dummy_start->num_of_dead_agents++;

		/*if (paths[i]->at(2).conflist == NULL) {
			cout << "x" << endl;

		}
		else {
			cout << paths[i]->at(2).conflist->size() << endl;

		}*/



	}

	//printPaths();

	if (debug_mode)
		cout << "Initializing dummy start" << endl;
	// generate dummy start and update data structures	

	dummy_start->h_val = 0;
	dummy_start->f_val = dummy_start->g_val;
	dummy_start->depth = 0;

	dummy_start->open_handle = open_list.push(dummy_start);
	dummy_start->focal_handle = focal_list.push(dummy_start);

	HL_num_generated++;
	dummy_start->time_generated = HL_num_generated;
	allNodes_table.push_back(dummy_start);
	if (debug_mode)
		cout << "Find initial conflict" << endl;
	findConflicts(*dummy_start);
	if (debug_mode)
		cout << "Find initial conflict done" << endl;
	min_f_val = make_tuple(dummy_start->num_of_dead_agents, dummy_start->makespan, dummy_start->f_val);
	focal_list_threshold = make_tuple(dummy_start->num_of_dead_agents, dummy_start->makespan, (int)(dummy_start->f_val * focal_w));
	if (debug_mode)
	{

		cout << "Dummy start conflicts:";
		for (auto &conit : dummy_start->unknownConf) {
			cout << "<" << conit->a1<< "," << conit->a2 << ","
				<< "("  << ")" << ","
				<< "("  << ")" << ","
				<< conit->t<< ">; ";

		}
		cout << endl;
	}

	if (cons_strategy == constraint_strategy::CBSH_GR)
		mddTable.resize(num_of_agents);
	if(debug_mode)
		cout << "Initializing done" << endl;

}
/*bool updateAndReplan() {

};*/


//template<class Map>
//void MultiMapICBSSearch<Map>::buildMDD(ICBSNode& curr, int id)
//{
//	if (debug_mode)
//		cout << "start build MDD" << endl;
//	MDD<Map> * mdd = new MDD<Map>();
//
//	vector < list< pair<int, int> > >* cons_vec = collectConstraints(&curr, id);
//	mdd->buildMDD(*cons_vec, paths[id]->size(), *search_engines[id]);
//
//	for (int i = 0; i < mdd->levels.size(); i++)
//		paths[id]->at(i).single = mdd->levels[i].size() == 1;
//	if (cons_strategy == constraint_strategy::CBSH_RM)
//	{
//		for (int i = 0; i < mdd->levels.size(); i++)
//		{
//			for (MDDNode* n : mdd->levels[i])
//			{
//				paths[id]->at(i).locations.push_back(n->location);
//			}
//		}
//	}
//	delete mdd;
//	delete cons_vec;
//	if (debug_mode)
//		cout << "build MDD done" << endl;
//}

template<class Map>
MDD<Map>* MultiMapICBSSearch<Map>::buildMDD(ICBSNode& node, int id)
{
	MDD<Map>* mdd = NULL;
	if (!mddTable.empty())
	{
		ConstraintsHasher c(id, &node);
		typename std::unordered_map<ConstraintsHasher, MDD<Map>*>::const_iterator got = mddTable[c.a].find(c);
		if (got != mddTable[c.a].end())
		{
			mdd = got->second;
		}
	}
	if (mdd == NULL)
	{
		mdd = new MDD<Map>();
		// vector < list< pair<int, int> > >* cons_vec = collectConstraints(&node, id);
		updateConstraintTable(&node, id);
		mdd->buildMDD(constraintTable, paths[id]->size(), *search_engines[id]);
	}

	for (int i = 0; i < mdd->levels.size(); i++)
		paths[id]->at(i).single = mdd->levels[i].size() == 1;
	if (cons_strategy == constraint_strategy::CBSH_RM || cons_strategy == constraint_strategy::CBSH_GR)
	{
		for (int i = 0; i < mdd->levels.size(); i++)
		{
			for (MDDNode* n : mdd->levels[i])
			{
				paths[id]->at(i).locations.push_back(n->location);
			}
		}
	}
	if (!mddTable.empty())
	{
		ConstraintsHasher c(id, &node);
		mddTable[c.a][c] = mdd;
	}
	return mdd;
}

template<class Map>
bool MultiMapICBSSearch<Map>::findPathForSingleAgent(ICBSNode*  node, int ag, double lowerbound)
{
	if (debug_mode)
		cout << "Start LL search" << endl;
	// extract all constraints on agent ag
	ICBSNode* curr = node;
	// vector < list< pair<int, int> > >* cons_vec = collectConstraints(curr, ag);
	updateConstraintTable(curr, ag);
	// build reservation table
	size_t max_plan_len = node->makespan + 1;

	ReservationTable res_table(map_size, &paths, ag,this->max_malfunction,ignoreFinishedAgent);  // initialized to false
	// find a path
	vector<PathEntry> newPath;
	//cout << "**************" << endl;
	//cout << "Single agent : " << curr->agent_id << endl;
	// bool foundSol = search_engines[ag]->findPath(newPath, focal_w, constraintTable, &res_table, max_plan_len, lowerbound, start, time_limit);
	// TODO: for now, I use w=1 for the low-level, because
	//  if the low-level path is suboptimal, mdds, cardinal conflicts and many other parts need to be reconsidered.
    bool foundSol = search_engines[ag]->findPath(newPath, 1, constraintTable, &res_table, max_plan_len, lowerbound, start, time_limit);

	LL_num_expanded += search_engines[ag]->num_expanded;
	LL_num_generated += search_engines[ag]->num_generated;

    node->paths.emplace_back(ag, newPath);
    if (foundSol)
    {
        assert(!newPath.empty());
        node->g_val = node->g_val - paths[ag]->size() + newPath.size();
        paths[ag] = &node->paths.back().second;
        node->makespan = std::max(node->makespan, (int)newPath.size() - 1);
    }
    else
    {
        node->num_of_dead_agents++;
        node->g_val = node->g_val - paths[ag]->size() + 1;
        paths[ag] = &node->paths.back().second;
        node->makespan = 0;
        for (const auto& path : paths)
            node->makespan = max(node->makespan, (int)path->size() - 1);
    }


	return true;
}


template<class Map>
void MultiMapICBSSearch<Map>::updateConstraintTable(ICBSNode* curr, int agent_id)
{
	constraintTable.copy(al.constraintTable);
	constraintTable.goal_location = search_engines[agent_id]->goal_location;
	while (curr != dummy_start)
	{
		if (curr->agent_id == agent_id)
		{
			for (auto constraint : curr->constraints)
			{
				int x, y, z;
				constraint_type type;
				tie(x, y, z, type) = constraint;
				if (type == constraint_type::RANGE) // time range constraint
				{
					constraintTable.insert(x, y, z + 1);
				}
				else if (type == constraint_type::BARRIER) // barrier constraint
				{
					int x1 = x / num_col, y1 = x % num_col;
					int x2 = y / num_col, y2 = y % num_col;
					if (x1 == x2)
					{
						if (y1 < y2)
							for (int i = 0; i <= std::min(y2 - y1, z); i++)
							{
								constraintTable.insert(x1 * num_col + y2 - i, z - i, 0);
							}
						else
							for (int i = 0; i <= std::min(y1 - y2, z); i++)
							{

								constraintTable.insert(x1 * num_col + y2 + i, z - i, 0);
							}
					}
					else // y1== y2
					{
						if (x1 < x2)
							for (int i = 0; i <= std::min(x2 - x1, z); i++)
							{

								constraintTable.insert((x2 - i) * num_col + y1, z - i, 0);
							}
						else
							for (int i = 0; i <= std::min(x1 - x2, z); i++)
							{

								constraintTable.insert((x2 + i) * num_col + y1, z - i, 0);
							}
					}
				}
				else if (type == constraint_type::LENGTH)
				{
					if (x < 0 && y == agent_id)
					{ // <-1, agent_id, t>: path of agent_id should be of length at least t + 1 
						constraintTable.length_min = max(constraintTable.length_min, z + 1);

					}
					else if (x >= 0 && y == agent_id)
					{ // <loc, agent_id, t>: path of agent_id should be of length at most t !!not possible as y==agent will jump findpath
						constraintTable.length_max = min(constraintTable.length_max, z);

					}
					else if (x >= 0 && y != agent_id)
					{ // <loc, agent_id, t>: any other agent cannot be at loc at or after timestep t
						constraintTable.insert(x, z, MAX_COST);
					}
				}
				else if (type == constraint_type::VERTEX)
				{
					constraintTable.insert(x, z, 0);
				}
				else // edge
				{
					constraintTable.insert(x * map_size + y, z, 0);
				}
			}
		}
		else if (!curr->constraints.empty())
		{
			int x, y, z;
			constraint_type type;
			tie(x, y, z, type) = curr->constraints.front();
			if (type == constraint_type::LENGTH && x >= 0 && y != agent_id)
			{
				constraintTable.insert(x, z, MAX_COST);
			}
		}
		curr = curr->parent;
	}
}

template<class Map>
void MultiMapICBSSearch<Map>::classifyConflicts(ICBSNode &parent)
{
	if (parent.conflicts.empty() && parent.unknownConf.empty())
		return; // No conflict
	int previousRetangle[4] = {0,0,0,0};
	// Classify all conflicts in unknownConf
	while (!parent.unknownConf.empty())
	{
		std::shared_ptr<Conflict> con = parent.unknownConf.front();

		int a1 = con->a1, a2 = con->a2;
		int loc1, loc2, timestep,timestep2;
		conflict_type type = con->type;
		loc1 = con->originalConf1;
		loc2 = con->originalConf2;

		timestep = con->t;
		timestep2 = timestep + con->k;
		//std::tie(loc1, loc2, timestep, type) = con->constraint1.front();
		parent.unknownConf.pop_front();
        if(debug_mode){
            cout<<"Classify: "<<"<"<<a1<<","<<a2<<","<<loc1 <<","<<loc2<<","<<timestep<<","<<con->k<<","<<type<<">";
        }
		
		bool cardinal1 = false, cardinal2 = false;
		if (timestep >= paths[a1]->size())
			cardinal1 = true;
		else if (!paths[a1]->at(0).single)
		{
			MDD<Map>* mdd = buildMDD(parent, a1);
			for (int i = 0; i < mdd->levels.size(); i++)
				paths[a1]->at(i).single = mdd->levels[i].size() == 1;
			if (mddTable.empty())
				delete mdd;
		}
		if (timestep2 >= paths[a2]->size())
			cardinal2 = true;
		else if (!paths[a2]->at(0).single)
		{
			MDD<Map>* mdd = buildMDD(parent, a2);
			for (int i = 0; i < mdd->levels.size(); i++)
				paths[a2]->at(i).single = mdd->levels[i].size() == 1;
			if (mddTable.empty())
				delete mdd;
		}

		if (type == conflict_type::STANDARD && loc2 >= 0) // Edge conflict
		{
			cardinal1 = paths[a1]->at(timestep).single && paths[a1]->at(timestep - 1).single;
			cardinal2 = paths[a2]->at(timestep2).single && paths[a2]->at(timestep2 - 1).single;
		}
		else // vertex conflict or target conflict
		{
			if (!cardinal1)
				cardinal1 = paths[a1]->at(timestep).single;
			if (!cardinal2)
				cardinal2 = paths[a2]->at(timestep2).single;
		}

		if (cardinal1 && cardinal2)
		{
			con->p = conflict_priority::CARDINAL;
			if (debug_mode)
			    cout << " --- cardinal" << endl;
		}
		else if (cardinal1 || cardinal2)
		{
			con->p = conflict_priority::SEMI;
            if (debug_mode)
                cout << " --- semi-cardinal" << endl;
		}
		else
		{
			con->p = conflict_priority::NON;
            if (debug_mode)
                cout << " --- non-cardinal" << endl;
		}
		if (con->p == conflict_priority::CARDINAL && cons_strategy == constraint_strategy::ICBS)
		{
			parent.conflicts.push_back(con);
			return;
		}

		if (con->type == conflict_type::TARGET)
		{
			parent.conflicts.push_back(con);
			continue;
		}

		// Corridor reasoning
		std::shared_ptr<Conflict> corridor;
		if ((corridor2 || corridor4) && isCorridorConflict(corridor, con, cardinal1 && cardinal2, &parent))
		{
			

			corridor->p = con->p;
			parent.conflicts.push_back(corridor);
			continue;
		}
		if (con->type == conflict_type::STANDARD &&
			paths[con->a1]->size() <= con->t || paths[con->a2]->size() <= con->t) //conflict happens after agent reaches its goal
		{
			parent.conflicts.push_back(con);
			continue;
		}

		
		parent.conflicts.push_back(con);
	}

	// remove conflicts that cannot be chosen, to save some memory
	removeLowPriorityConflicts(parent.conflicts);
}

template<class Map>
int MultiMapICBSSearch<Map>::getBestSolutionSoFar()
{
    // find the best node
    auto best = focal_list.top();
    /*for (auto node : open_list)
    {
        if (node->num_of_dead_agents < best->num_of_dead_agents ||
            (node->num_of_dead_agents == best->num_of_dead_agents &&
            node->num_of_collisions < best->num_of_collisions))
            best = node;
    }*/

    // Find a maximal independent set of paths (i.e., collision-free paths)
    int num_of_giveup_agents = 0;
    updatePaths(best);
    for (const auto& conflict : best->conflicts)
    {
        if (paths[conflict->a1] != nullptr && paths[conflict->a2] != nullptr)
        { // give up the agent with shorter path
            if (paths[conflict->a1]->size() < paths[conflict->a2]->size())
                paths[conflict->a1] = nullptr;
            else
                paths[conflict->a2] = nullptr;
            num_of_giveup_agents++;
        }
    }
    for (const auto& conflict : best->unknownConf)
    {
        if (paths[conflict->a1] != nullptr && paths[conflict->a2] != nullptr)
        { // give up the agent with shorter path
            if (paths[conflict->a1]->size() < paths[conflict->a2]->size())
                paths[conflict->a1] = nullptr;
            else
                paths[conflict->a2] = nullptr;
            num_of_giveup_agents++;
        }
    }
    return num_of_giveup_agents;
}

template class MultiMapICBSSearch<MapLoader>;
template class MultiMapICBSSearch<FlatlandLoader>;
