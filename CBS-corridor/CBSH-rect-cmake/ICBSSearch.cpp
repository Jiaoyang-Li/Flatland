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

	while (curr->parent != nullptr)
	{
		for (auto& path_pair :curr->paths)
		{
			if (!updated[path_pair.first])
			{
				paths[path_pair.first] = &(path_pair.second);
				updated[path_pair.first] = true;
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
	for (const auto& conflict : curr.conflicts)
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
	if (curr.parent == nullptr || // root node of CBS tree or
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
            for (int k = 1; k < (int)indices.size(); k++)
            {
                if (KVertexCover(subgraph, (int)indices.size(), num_edges, k))
                {
                    rst += k;
                    break;
                }
                runtime = (double)(std::clock() - start);
                if (runtime > time_limit)
                    return -1; // run out of time
            }
        }
        else
        {
            rst += greedyMatching(subgraph);
        }
    }
    return rst;
}

int ICBSSearch::greedyMatching(const vector<vector<bool>>& CG)
{
    int rst = 0;
    std::vector<bool> selected(CG.size(), false);
    for (int i = 0; i < (int)CG.size(); i++)
    {
        if (selected[i])
            continue;
        for (int j = i + 1; j < (int)CG.size(); j++)
        {
            if(CG[i][j] && !selected[j])
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
	for (int i : node)
	{
		vector<vector<bool>> CG_copy(CG.size());
		CG_copy.assign(CG.cbegin(), CG.cend());
		int num_of_CGedges_copy = num_of_CGedges;
		for (int j = 0; j < (int)CG.size(); j++)
		{
			if (CG_copy[i][j])
			{
				CG_copy[i][j] = false;
				CG_copy[j][i] = false;
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
	std::list<std::shared_ptr<Conflict>>& copy, const list<int>& excluded_agents)
{
	for (const auto& conflict : conflicts)
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

	if (curr.parent != nullptr)
	{
		if (debug_mode)
			cout << "copy from parent" << endl;
		// Copy from parent、
		list<int> new_agents;
		for (const auto& p : curr.paths)
		{
			new_agents.push_back(p.first);
		}
		copyConflicts(curr.parent->conflicts,curr.conflicts, new_agents);
        copyConflicts(curr.parent->unknownConf, curr.unknownConf, new_agents);
        copyConflicts(curr.parent->cardinal_wating, curr.cardinal_wating, new_agents);
        copyConflicts(curr.parent->non_cardinal_wating, curr.non_cardinal_wating, new_agents);

		if (debug_mode) {
            cout << "Find Conflicts" << endl;
            cout << "Number of existing unknown: "<<curr.unknownConf.size()<<endl;
        }
		// detect new conflicts
		for (auto it = new_agents.begin(); it != new_agents.end(); ++it)
		{
			int a1 = *it;
			//collect conflict from path;
			for (auto& state : *paths[a1])
			{
				if (state.conflist == nullptr)
				    continue;
                for (const auto& con : *(state.conflist))
                {
                    if (debug_mode)
                        cout << "l<" << get<0>(*con) << "," << get<1>(*con) << ","
                        << "(" << get<2>(*con) / num_col << "," << get<2>(*con) % num_col << ")" << ","
                        << "(" << get<3>(*con) / num_col << "," << get<3>(*con) % num_col << ")" << ","
                        << get<4>(*con) << "," << get<5>(*con) << ">; ";

                    std::shared_ptr<Conflict> newConf(new Conflict());
                    newConf->vertexConflict(get<0>(*con), get<1>(*con), get<2>(*con), get<4>(*con), get<5>(*con), kDelay);
                    curr.unknownConf.emplace_back(newConf);

                }
                delete state.conflist;
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
            for (auto& state : *paths[a1])
            {
				if (state.conflist == nullptr)
					continue;
				for (auto& con : *(state.conflist))
				{
					std::shared_ptr<Conflict> newConf(new Conflict());
					newConf->vertexConflict(get<0>(*con), get<1>(*con), get<2>(*con), get<4>(*con), get<5>(*con), kDelay);
					if (debug_mode)
                        cout << "<" << get<0>(*con) << "," << get<1>(*con) << ","
                            << "(" << get<2>(*con) / num_col << "," << get<2>(*con) % num_col << ")" << ","
                            << "(" << get<3>(*con) / num_col << "," << get<3>(*con) % num_col << ")" << ","
                            << get<4>(*con) << "," << get<5>(*con) << ">; ";
					curr.unknownConf.emplace_back(newConf);
				}
				delete state.conflist;
			}
		}
	}
	if (debug_mode)
		cout  << endl;
}

/*void ICBSSearch::findTargetConflicts(int a1, int a2, ICBSNode& curr) {
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
}*/

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
bool MultiMapICBSSearch<Map>::isCorridorConflict(std::shared_ptr<Conflict>& corridor, const std::shared_ptr<Conflict>& con, ICBSNode* node)
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
	t[0] = cp.getEnteringTime(*paths[a[0]], *paths[a[1-0]], timestep, ml)+1; //+1 is the inside vertex of the corridor, no matter what speed
	t[1] = cp.getEnteringTime(*paths[a[1]], *paths[a[1 - 1]], timestep+kConflict, ml)+1;

	if (t[0] > t[1])
	{
		int temp = t[0]; t[0] = t[1]; t[1] = temp;
		temp = a[0]; a[0] = a[1]; a[1] = temp;
	}

	bool chasing = false;
	int u[2];//get entering location

    for (int i = 0; i < 2; i++) {
        if (t[i] >= paths[a[i]]->size())
            return false;
        u[i] = paths[a[i]]->at(t[i]).location;
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

	if (chasing && chasing_reasoning) {
	    if (debug_mode)
	        cout<<"chasing"<<endl;

        //get exit location
        int e[2];
        e[0] = cp.getExitTime(*paths[a[0]], *paths[a[1]], timestep, ml)-1; //-1 is the inner vertex of the corridor no matter what speed.
        e[1] = cp.getExitTime(*paths[a[1]], *paths[a[0]], timestep+kConflict, ml)-1;
        int el[2];
        el[0] = paths[a[0]]->at(e[0]).location;
        el[1] = paths[a[1]]->at(e[1]).location;


        for(int i = -kDelay;i <= kDelay;i++){
            if((e[0]+i==e[1] && t[0]+i==t[1]) ){
                if(debug_mode)
                    cout<<"same path"<<endl;
                return false;
            }

        }

        if(el[0]==el[1]) {

            MDD<Map>* a0mdd = buildMDD(*node, a[0]);
            MDD<Map>* a1mdd = buildMDD(*node, a[1]);

            //get exit time for a1
            int a0exit = a0mdd->getTime(el[0],(*paths[a[0]])[e[0]].heading);

            int a0entrance = a0mdd->getTime(u[0],(*paths[a[0]])[t[0]].heading);


            //get exit time for a2
            int a1exit =  a1mdd->getTime(el[1],(*paths[a[1]])[e[1]].heading);

            int a1entrance = a1mdd->getTime(u[1],(*paths[a[1]])[t[1]].heading);
//            if (a1entrance < 0 || a1exit<0) {
//                a2mdd->print();
//                cout<<(*paths[a[1]])[t[1]].heading<<endl;
//            }
            assert(a0exit>=0 && a1exit>=0 && a0entrance>=0 && a1entrance>=0);
            if(!((a0exit <= a1exit && a0entrance >= a1entrance)||(a0exit >= a1exit && a0entrance <= a1entrance))){
                return false;
            }
            int earlyExitAgent;
            int early_exit;
            int early_entrance;
            int lateExitAgent;
            int late_entrance;
            int late_exit;
            if (a0exit <= a1exit) {
                earlyExitAgent = a[0];
                lateExitAgent = a[1];
                early_entrance = a1entrance;
                late_entrance = a0entrance + 1/al.agents[a[0]]->speed;
                early_exit = a0exit;
                late_exit = a1exit + 1/al.agents[a[1]]->speed;
            } else {
                earlyExitAgent = a[1];
                lateExitAgent = a[0];
                early_entrance = a0entrance;
                late_entrance = a1entrance + 1/al.agents[a[1]]->speed;
                early_exit = a1exit;
                late_exit = a0exit + 1/al.agents[a[0]]->speed;

            }
            if(late_entrance>=early_entrance && late_exit>=early_exit){
                corridor = std::make_shared<Conflict>(); // std::shared_ptr<Conflict>(new Conflict());
                corridor->chasingConflict(earlyExitAgent, lateExitAgent, loc1, loc2,u[0], el[0],
                        late_entrance,late_entrance,early_exit, late_exit, kDelay);

                if (blocked(*(paths[corridor->a1]), corridor->constraint1) && blocked(*(paths[corridor->a2]), corridor->constraint2)) {
                    if (debug_mode)
                        cout<<"blocked"<<endl;
                    return true;
                }
                else {
                    if (debug_mode) {
                        cout << "not blocked" << endl;
                        cout<< "conflict time 0: "<<timestep <<" 1: "<<timestep+kConflict<<endl;
                        cout << late_entrance << "," << early_entrance << "," << late_exit << "," << early_exit << endl;
                    }
                    return false;
                }
            }
            else{
                if(debug_mode) {
                    cout << late_entrance << "," << early_entrance << "," << late_exit << "," << early_exit << endl;
                    cout << "not crossing" << endl;
                }
                return false;
            }
        }

	}

    if(chasing)
        return false;
    if (trainCorridor1 || corridor2){
        if(debug_mode)
        cout<<"train corridor"<<endl;


        int k = getCorridorLength(*paths[a[0]], t[0]-1, u[1], edge)+1;

        int t0 = t[0];
        int t1 = t[1];
        int exit_t0 = t1 + k/al.agents[a[1]]->speed + kDelay;
        int exit_t1 = t0 + k/al.agents[a[0]]->speed + kDelay;

        corridor = make_shared<Conflict>();
        corridor->trainCorridorConflict(a[0], a[1], u[0], u[1], t0, t1,exit_t0, exit_t1);
        if (blocked(*(paths[corridor->a1]), corridor->constraint1) &&
            blocked(*(paths[corridor->a2]), corridor->constraint2)) {
            if (debug_mode)
                cout << "is blocked" << endl;
            return true;

        }
        if (debug_mode) {
            cout << "not blocked" << endl;
        }
        return false;
    }

	return false;

}



void ICBSSearch::removeLowPriorityConflicts(std::list<std::shared_ptr<Conflict>>& conflicts) const
{
	if (conflicts.empty())
		return;
	std::unordered_map<int, std::shared_ptr<Conflict> > keep;
	std::list<std::shared_ptr<Conflict>> to_delete;
	for (auto& conflict : conflicts)
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

	for (auto& conflict : to_delete)
	{
		conflicts.remove(conflict);
	}
}


bool ICBSSearch::traverse(const Path& path, int loc, int t)
{
    assert(t < path.size() && t >= 0);
	return (path[t].location == loc);
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
		/*else if (type == constraint_type::BARRIER) // barrier constraint
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
		}*/
		else {
			if (traverse(path, x, t))
				return true;
		}
	}
	return false;
}


std::shared_ptr<Conflict> ICBSSearch::chooseConflict(ICBSNode &parent) const
{
	if (debug_mode)
		cout << "Start choosing conflict" << endl;

	std::shared_ptr<Conflict> choose;
	if (parent.conflicts.empty() && parent.unknownConf.empty())
		return nullptr;
	else if (!parent.conflicts.empty())
	{
		choose = parent.conflicts.back();
		for (auto& conflict : parent.conflicts)
		{
			if (*choose < *conflict)
				choose = conflict;
		}
	}
	else
	{
		choose = parent.unknownConf.back();
		for (auto& conflict : parent.unknownConf)
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
    double lowerbound = (int)paths[node->agent_id]->size() - 1;
    // if (curr->conflict->p == conflict_priority::CARDINAL && curr->conflict->type != conflict_type::CORRIDOR2)
    //	lowerbound += 1;

    if (!findPathForSingleAgent(node, node->agent_id, lowerbound))
        return false;




	

	
	runtime_lowlevel += (double)(std::clock() - t1) * 1000.0 / CLOCKS_PER_SEC;
	
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
	runtime_conflictdetection += (double)(std::clock() - t1)  * 1000.0 / CLOCKS_PER_SEC;

	node->num_of_collisions = node->unknownConf.size() + node->conflicts.size();

	// update handles
	node->open_handle = open_list.push(node);
	HL_num_generated++;
	node->time_generated = HL_num_generated;
	if(debug_mode) {
        cout << "child dead: " <<node->num_of_dead_agents << " makespan: "<<node->makespan << "f: " << node->f_val<< endl;
        cout << "focal_list_threshold: " << get<0>(focal_list_threshold)<<","<< get<1>(focal_list_threshold) <<","<<get<2>(focal_list_threshold) << endl;
    }
	if (node->num_of_dead_agents == get<0>(focal_list_threshold) &&
	    node->makespan <= get<1>(focal_list_threshold) &&
	    node->f_val <= get<2>(focal_list_threshold))
	{
	    if(debug_mode)
	        cout<<"add to focal"<<endl;
        node->focal_handle = focal_list.push(node);
    }
	allNodes_table.push_back(node);


	return true;
}

/*bool ICBSSearch::hasCardinalConflict(const ICBSNode& node) const
{
	for (auto conflict : node.conflicts)
	{
		if (conflict->p = conflict_priority::CARDINAL)
			return true;
	}
	return false;
}*/


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

		for (const auto & state : path)
			std::cout << "(" << state.location / num_col << "," << state.location % num_col << ")->";
		std::cout << std::endl;
}

void ICBSSearch::printBT(const std::string& prefix, const ICBSNode* node, bool isLeft)
{
	if (node != nullptr)
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
			printBT(prefix + (isLeft ? "│   " : "    "), node->children[i], i != node->children.size() - 1);
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
    focal_list_threshold = make_tuple(open_head->num_of_dead_agents,
            max(max(al.makespan, open_head->makespan), al.constraintTable.length_max / 2), // latest_timestep is the makespan of the planned paths in previous iterations
            (int)(open_head->f_val * focal_w));
    focal_list.clear();
	for (ICBSNode* n : open_list) {
        if (n->num_of_dead_agents == get<0>(focal_list_threshold) &&
            n->makespan <= get<1>(focal_list_threshold) &&
            n->f_val <= get<2>(focal_list_threshold))
			n->focal_handle = focal_list.push(n);
	}

}

void ICBSSearch::updateReservationTable(bool* res_table, int exclude_agent, const ICBSNode &node) {
	for (int ag = 0; ag < num_of_agents; ag++) 
	{
		if (ag != exclude_agent && paths[ag] != nullptr)
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
		if (curr->parent == nullptr)
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
		else if(curr->conflict == nullptr) //CBSH based, and h value has not been computed yet
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

            if (curr->num_of_dead_agents > get<0>(focal_list_threshold) ||
                curr->makespan > get<1>(focal_list_threshold) ||
                curr->f_val > get<2>(focal_list_threshold))
			{
				t1 = std::clock();
				curr->open_handle = open_list.push(curr);
				runtime_listoperation += std::clock() - t1;
				continue;
			}

		}



		if (curr->conflict == nullptr) //Fail to find a conflict => no conflicts
		{  // found a solution (and finish the while look)
			runtime = (std::clock() - start);
			solution_found = true;
			solution_cost = curr->g_val;
            goal_node = curr;
			collectConstraints(curr);

			if (screen>=1)
				printHLTree();
			if (screen >= 2)
				printPaths();
			assert(solution_cost >= dummy_start->g_val);
			cout << solution_cost << " (" << goal_node->num_of_dead_agents << ") ; " << goal_node->makespan << " ; " <<
			    solution_cost - dummy_start->g_val << " ; " <<
				HL_num_expanded << " ; " << HL_num_generated << " ; " <<
				LL_num_expanded << " ; " << LL_num_generated << " ; " << runtime / CLOCKS_PER_SEC << ";"<<
				num_standard << ";" << num_start << "," <<
				num_corridor2 << ";" << num_chasing << "," << num_corridor << endl;
			
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

		if (screen>=1) {
		    if(debug_mode)
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
            if (parent != nullptr) {
                if (debug_mode)
                    cout << "Try find " << con.str() << " in curr's parent nodes" << endl;
                while (!stop) {
                    if (debug_mode)
                        cout << "1";
                    if (parent->parent == nullptr) {
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
		else // 2-way branching
		{
			
			children.resize(2);
			children[0] = new ICBSNode(curr->conflict->a1);
			children[0]->constraints = curr->conflict->constraint1;
			children[1] = new ICBSNode(curr->conflict->a2);
			children[1]->constraints = curr->conflict->constraint2;
			if (curr->conflict->type == conflict_type::CORRIDOR2)
            {
                num_corridor2++;
            }
			else if (curr->conflict->type == conflict_type::STANDARD) {
                num_standard++;
                if(get<0>(curr->conflict->constraint1.front()) == search_engines[curr->conflict->a1]->start_location
                || get<0>(curr->conflict->constraint2.front()) == search_engines[curr->conflict->a2]->start_location)
                    num_activeConflict++;
            }
            else if (curr->conflict->type == conflict_type::CHASING)
            {
                num_chasing++;
            }
            else if (curr->conflict->type == conflict_type::CORRIDOR)
            {
                num_corridor++;
            }
            else if (curr->conflict->type == conflict_type::START)
            {
                num_start++;
            }
		}

		if (screen >= 2)
		{
			std::cout << "Node " << curr->time_generated << " with f=" << curr->g_val << "+" << curr->h_val
				<< " has " << std::endl;  
			for (const auto& conflict : curr->conflicts)
				std::cout << *conflict;
			std::cout << "We choose " << *curr->conflict;
		}
		vector<vector<PathEntry>*> copy(paths);
		for (int i = 0; i < children.size(); i++)
		{
			if (!children[i]->constraints.empty() && generateChild(children[i], curr))
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
		if (std::clock() - start > time_limit)
        {
            curr->open_handle = open_list.push(curr); // to make sure that the open list has at least one node
            focal_list.push(curr);
            break;
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
        if (screen>=1)
            printHLTree();
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
             << num_standard << ";" << num_start << "," <<
                     num_corridor2 << ";" << num_chasing << "," << num_corridor << endl;
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
	for (auto & it : allNodes_table)
		delete it;
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
        int time_limit, int screen, options options1): ICBSSearch(*al0)
{
	this->option = options1;
	this->focal_w = f_w;
	this->time_limit = time_limit;
	this->screen = screen;
	if (screen >= 3)
		debug_mode = true;
    else
        debug_mode = false;


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
	kDelay = 1;
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
	paths.resize(num_of_agents, nullptr);
	paths_found_initially.resize(num_of_agents);
	constraintTable.clear();
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
        bool found = search_engines[i]->findPath(paths_found_initially[i], 1,  // focal_w,
                max(max(al.makespan, dummy_start->makespan), constraintTable.length_max / 2),
                                                 constraintTable,
                                                 &res_table, dummy_start->makespan + 1, 0);
        LL_num_expanded += search_engines[i]->num_expanded;
        LL_num_generated += search_engines[i]->num_generated;
        paths[i] = &paths_found_initially[i];
        if (found)
        {
            res_table.addPath(i, *paths[i]);
            dummy_start->makespan = max(dummy_start->makespan, (int)paths_found_initially[i].size() - 1);
            dummy_start->g_val += paths[i]->size() - 1;
        }
		else
        {
            dummy_start->num_of_dead_agents++;
        }




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
	focal_list_threshold = make_tuple(dummy_start->num_of_dead_agents,
	        max(max(al.makespan, dummy_start->makespan), al.constraintTable.length_max / 2),  // latest_timestep is the makespan of the planned paths in previous iterations
	        (int)(dummy_start->f_val * focal_w));
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
	MDD<Map>* mdd = nullptr;
	if (!mddTable.empty())
	{
		ConstraintsHasher c(id, &node);
		typename std::unordered_map<ConstraintsHasher, MDD<Map>*>::const_iterator got = mddTable[c.a].find(c);
		if (got != mddTable[c.a].end())
		{
			mdd = got->second;
		}
	}
	if (mdd == nullptr)
	{
		mdd = new MDD<Map>();
		// vector < list< pair<int, int> > >* cons_vec = collectConstraints(&node, id);
		updateConstraintTable(&node, id);
		mdd->buildMDD(constraintTable, paths[id]->size(), *search_engines[id]);
	}

	for (int i = 0; i < mdd->levels.size(); i++)
		paths[id]->at(i).single = mdd->levels[i].size() == 1;
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
    bool foundSol = search_engines[ag]->findPath(newPath,  1, // focal_w,
                    get<1>(focal_list_threshold),
                    al.constraintTable, &res_table, max_plan_len, lowerbound, start, time_limit);

	LL_num_expanded += search_engines[ag]->num_expanded;
	LL_num_generated += search_engines[ag]->num_generated;
    if ((std::clock() - start) > time_limit) // run out of time
    {
        return false;
    }
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
	constraintTable.clear();
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
				else if (type == constraint_type::VERTEX)
				{
					constraintTable.insert(x, z, 0);
				}
				//else // edge
				//{
				//	constraintTable.insert(x * map_size + y, z, 0);
				//}
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
			break;
		}
		int a1start = al.agents[a1]->initial_location.first * num_col + al.agents[a1]->initial_location.second;
        int a2start = al.agents[a2]->initial_location.first * num_col + al.agents[a2]->initial_location.second;

        if(loc1 == a1start && loc1 == a2start){
            con->type = conflict_type::START;
        }

		if(con->p == conflict_priority::CARDINAL){
		    parent.cardinal_wating.push_back(con);
		}
		else{
		    parent.non_cardinal_wating.push_back(con);
		}


		parent.conflicts.push_back(con);
	}
    if (!corridor2)
        return;
    double corridorT = std::clock();
    bool found = false;
    while ( !parent.cardinal_wating.empty()){
        std::shared_ptr<Conflict> conflict  = parent.cardinal_wating.front();
        parent.cardinal_wating.pop_front();
        // Corridor reasoning
        std::shared_ptr<Conflict> corridor;
        if( isCorridorConflict(corridor, conflict, &parent))
        {

            corridor->p = conflict->p;
            parent.conflicts.push_back(corridor);
            parent.conflicts.remove(conflict);

            found = true;
            break;
        }
	}

    while ( !parent.non_cardinal_wating.empty() && !found){
        std::shared_ptr<Conflict> conflict  = parent.non_cardinal_wating.front();
        parent.non_cardinal_wating.pop_front();
        std::shared_ptr<Conflict> corridor;
        if( isCorridorConflict(corridor, conflict, &parent))
        {

            corridor->p = conflict->p;
            parent.conflicts.push_back(corridor);
            parent.conflicts.remove(conflict);
            break;
        }
    }
    runtime_corridor += std::clock() - corridorT;


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
