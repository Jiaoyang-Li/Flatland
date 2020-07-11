#include "Conflict.h"

std::ostream& operator<<(std::ostream& os, const Constraint& constraint)
{
	os << "<" << std::get<0>(constraint) << "," << std::get<1>(constraint) << "," <<
		std::get<2>(constraint) << "," << std::get<3>(constraint) << ">";
	return os;
}


std::ostream& operator<<(std::ostream& os, const Conflict& conflict)
{
	switch (conflict.p)
	{
		case conflict_priority::CARDINAL:
			os << "cardinal ";
			break;
		case conflict_priority::SEMI:
			os << "semi-cardinal ";
			break;
		case conflict_priority::NON:
			os << "non-cardinal ";
			break;
	}

	switch (conflict.type)
	{
		case conflict_type::STANDARD:
			os << "standard";
			break;
		case conflict_type::CORRIDOR2:
			os << "corrdior2";
			break;
		case conflict_type::CORRIDOR4:
			os << "corrdior4";
			break;
	}
	os << " conflict:  " << conflict.a1 << " with ";
	for (auto con : conflict.constraint1)
		os << con << ",";		
	os << " and " << conflict.a2 << " with ";
	for (auto con : conflict.constraint2)
		os << con << ",";		
	os << std::endl;
	return os;
}

bool operator < (const Conflict& conflict1, const Conflict& conflict2) // return true if conflict2 has higher priority
{
//	if (conflict1.type == conflict_type::START && conflict2.type == conflict_type::START)
//	{
//		if (conflict1.p < conflict2.p)
//			return false;
//		else
//			return true;
//	}
//	else if (conflict1.type == conflict_type::START)
//		return false;
//	else if (conflict2.type == conflict_type::START)
//		return true;


//	if (conflict1.type == conflict_type::CORRIDOR2 && conflict2.type != conflict_type::CORRIDOR2)
//	{
//		return false;
//	}
//	else if (conflict2.type == conflict_type::CORRIDOR2 && conflict1.type != conflict_type::CORRIDOR2)
//	{
//		return true;
//	}
//    if (conflict1.type == conflict_type::CHASING && conflict2.type != conflict_type::CHASING)
//    {
//        return false;
//    }
//    else if (conflict2.type == conflict_type::CHASING && conflict1.type != conflict_type::CHASING)
//    {
//        return true;
//    }
//    if (conflict1.type == conflict_type::SEMI_CORRIDOR && conflict2.type != conflict_type::SEMI_CORRIDOR)
//    {
//        return false;
//    }
//    else if (conflict2.type == conflict_type::SEMI_CORRIDOR && conflict1.type != conflict_type::SEMI_CORRIDOR)
//    {
//        return true;
//    }

    if (conflict1.p < conflict2.p)
		return false;
	else if (conflict1.p > conflict2.p)
		return true;
	else if (conflict1.p == conflict_priority::CARDINAL) // both are cardinal
	{
		if (conflict1.type == conflict_type::CORRIDOR2)
		{
			if (conflict2.type != conflict_type::CORRIDOR2)
				return false;
		}
        else if (conflict1.type == conflict_type::CHASING)
        {
            if (conflict2.type != conflict_type::CHASING)
                return false;
        }
        else if (conflict1.type == conflict_type::SEMI_CORRIDOR)
        {
            if (conflict2.type != conflict_type::SEMI_CORRIDOR)
                return false;
        }
		else if (conflict2.type == conflict_type::CORRIDOR2 && conflict2.type != conflict_type::CHASING )
		{
			return true;
		}
        else if (conflict2.type == conflict_type::CHASING )
        {
            return true;
        }
        else if (conflict2.type == conflict_type::SEMI_CORRIDOR )
        {
            return true;
        }
	}
	else // both are semi or both are non 
	{

		if (conflict2.type == conflict_type::CORRIDOR2 &&  conflict1.type != conflict_type::CORRIDOR2)
		{
			return true;
		}
        else if (conflict2.type == conflict_type::CHASING &&  conflict1.type != conflict_type::CHASING)
        {
            return true;
        }
        else if (conflict2.type == conflict_type::SEMI_CORRIDOR &&  conflict1.type != conflict_type::SEMI_CORRIDOR)
        {
            return true;
        }
		else	if (conflict2.type != conflict_type::CORRIDOR2 &&  conflict1.type == conflict_type::CORRIDOR2)
		{
			return false;
		}
		else if(conflict2.type != conflict_type::CHASING && conflict1.type == conflict_type::CHASING){
		    return false;
		}
        else if(conflict2.type != conflict_type::SEMI_CORRIDOR && conflict1.type == conflict_type::SEMI_CORRIDOR){
            return false;
        }
		
		/*if (conflict2.type == conflict_type::RECTANGLE &&  conflict1.type != conflict_type::RECTANGLE)
		{
			return true;
		}*/
	}



	if (conflict2.t < conflict1.t)
	{
		return true;
	}
	else
		return false;
}

/*
// add a pair of barrier constraints
void addBarrierConstraints(int S1, int S2, int S1_t, int S2_t, int Rg, int num_col,
	std::list<Constraint>& constraints1, std::list<Constraint>& constraints2)
{
	int s1_x = S1 / num_col;
	int s1_y = S1 % num_col;
	int s2_x = S2 / num_col;
	int s2_y = S2 % num_col;
	int Rg_x = Rg / num_col;
	int Rg_y = Rg % num_col;
	int Rg_t = S1_t + abs(Rg_x - s1_x) + abs(Rg_y - s1_y);

	int R1_x, R1_y, R2_x, R2_y;
	if (s1_x == s2_x)
	{
		if ((s1_y - s2_y) * (s2_y - Rg_y) >= 0)
		{
			R1_x = s1_x;
			R2_x = Rg_x;
			R1_y = Rg_y;
			R2_y = s2_y;
		}
		else
		{
			R1_x = Rg_x;
			R2_x = s2_x;
			R1_y = s1_y;
			R2_y = Rg_y;
		}
	}
	else if ((s1_x - s2_x)*(s2_x - Rg_x) >= 0)
	{
		R1_x = Rg_x;
		R2_x = s2_x;
		R1_y = s1_y;
		R2_y = Rg_y;
	}
	else
	{
		R1_x = s1_x;
		R2_x = Rg_x;
		R1_y = Rg_y;
		R2_y = s2_y;
	}

	constraints1.emplace_back(R1_x * num_col + R1_y, Rg, Rg_t, constraint_type::BARRIER);
	constraints2.emplace_back(R2_x * num_col + R2_y, Rg, Rg_t, constraint_type::BARRIER);
}

// add a pair of modified barrier constraints
bool addModifiedBarrierConstraints(const std::vector<PathEntry>& path1, const std::vector<PathEntry>& path2,
	int S1_t, int S2_t, int Rg, int num_col,
	std::list<Constraint>& constraints1, std::list<Constraint>& constraints2)
{
	int s1_x = path1[S1_t].location / num_col;
	int s1_y = path1[S1_t].location % num_col;
	int s2_x = path2[S2_t].location / num_col;
	int s2_y = path2[S2_t].location % num_col;
	int Rg_x = Rg / num_col;
	int Rg_y = Rg % num_col;
	int Rg_t = S1_t + abs(Rg_x - s1_x) + abs(Rg_y - s1_y);

	bool succ1, succ2;
	int R1_x, R1_y, R2_x, R2_y;
	if ((s1_x == s2_x && (s1_y - s2_y) * (s2_y - Rg_y) < 0) ||
		(s1_x != s2_x && (s1_x - s2_x)*(s2_x - Rg_x) >= 0))
	{
		R1_x = Rg_x;
		R2_x = s2_x;
		R1_y = s1_y;
		R2_y = Rg_y;
		succ1 = addModifiedHorizontalBarrierConstraint(path1, Rg_x, R1_y, Rg_y, Rg_t, num_col, constraints1);
		succ2 = addModifiedVerticalBarrierConstraint(path2, Rg_y, R2_x, Rg_x, Rg_t, num_col, constraints2);
	}
	else
	{
		R1_x = s1_x;
		R2_x = Rg_x;
		R1_y = Rg_y;
		R2_y = s2_y;
		succ1 = addModifiedVerticalBarrierConstraint(path1, Rg_y, R1_x, Rg_x, Rg_t, num_col, constraints1);
		succ2 = addModifiedHorizontalBarrierConstraint(path2, Rg_x, R2_y, Rg_y, Rg_t, num_col, constraints2);
	}
	return succ1 && succ2;
}


// add a horizontal modified barrier constraint
bool addModifiedHorizontalBarrierConstraint(const std::vector<PathEntry>& path, int x,
	int Ri_y, int Rg_y, int Rg_t, int num_col,
	std::list<Constraint>& constraints)
{
	int sign = Ri_y < Rg_y ? 1 : -1;
	int Ri_t = Rg_t - abs(Ri_y - Rg_y);
	int t1 = -1;
	int t_min = std::max(Ri_t, 0);
	int t_max = std::min(Rg_t, (int)path.size() - 1);
	for (int t2 = t_min; t2 <= t_max; t2++)
	{
		int loc = (Ri_y + (t2 - Ri_t) * sign) + x * num_col;
		std::list<int>::const_iterator it = std::find(path[t2].locations.begin(), path[t2].locations.end(), loc);
		if (it == path[t2].locations.end() && t1 >= 0) // add constraints [t1, t2)
		{
			int loc1 = (Ri_y + (t1 - Ri_t) * sign) + x * num_col;
			int loc2 = (Ri_y + (t2 - 1 - Ri_t) * sign) + x * num_col;
			constraints.emplace_back(loc1, loc2, t2 - 1, constraint_type::BARRIER);
			t1 = -1;
			continue;
		}
		else if (it != path[t2].locations.end() && t1 < 0)
		{
			t1 = t2;
		}
		if (it != path[t2].locations.end() && t2 == t_max)
		{
			int loc1 = (Ri_y + (t1 - Ri_t) * sign) + x * num_col;
			constraints.emplace_back(loc1, loc, t2, constraint_type::BARRIER); // add constraints [t1, t2]
		}
	}
	if (constraints.empty())
	{
		// std::cout << "Fail to add modified barrier constraints!" << std::endl;
		return false;
	}
	else
		return true;
}

// add a pair of long k-delay barrier constraints
void addKDelayBarrierConstraints(int S1, int S2, int S1_t, int S2_t, int Rg, int G1, int G2, int num_col,
	std::list<Constraint>& constraints1, std::list<Constraint>& constraints2, int k, bool asymmetry_constraint)
{
	// 
	int s1_x = S1 / num_col;
	int s1_y = S1 % num_col;
	int s2_x = S2 / num_col;
	int s2_y = S2 % num_col;
	int g1_x = G1 / num_col;
	int g1_y = G1 % num_col;
	int g2_x = G2 / num_col;
	int g2_y = G2 % num_col;
	int Rg_x = Rg / num_col;
	int Rg_y = Rg % num_col;
	int Rg_t = S1_t + abs(Rg_x - s1_x) + abs(Rg_y - s1_y);


	int R1_x, R1_y, R2_x, R2_y;
	int Rg1_x, Rg1_y, Rg2_x, Rg2_y;
	if (s1_x == s2_x)
	{
		if ((s1_y - s2_y) * (s2_y - Rg_y) >= 0)
		{
			R1_x = s2_x;//different
			Rg1_x = g2_x;
			R2_x = Rg_x;
			Rg2_x = Rg_x;
			R1_y = Rg_y;
			Rg1_y = Rg_y;
			R2_y = s1_y;//different
			Rg2_y = g1_y;
		}
		else
		{
			R1_x = Rg_x;
			Rg1_x = Rg_x;
			R2_x = s1_x;//different
			Rg2_x = g1_x;
			R1_y = s2_y;//different
			Rg1_y = g2_y;
			R2_y = Rg_y;
			Rg2_y = Rg_y;
		}
	}
	else if ((s1_x - s2_x)*(s2_x - Rg_x) >= 0)
	{
		R1_x = Rg_x;
		Rg1_x = Rg_x;
		R2_x = s1_x;//different
		Rg2_x = g1_x;
		R1_y = s2_y;//different
		Rg1_y = g2_y;
		R2_y = Rg_y;
		Rg2_y = Rg_y;
	}
	else
	{
		R1_x = s2_x;//different
		Rg1_x = g2_x;
		R2_x = Rg_x;
		Rg2_x = Rg_x;
		R1_y = Rg_y;
		Rg1_y = Rg_y;
		R2_y = s1_y;//different
		Rg2_y = g1_y;
	}

	int Rg1_t = S1_t + abs(Rg1_x - s1_x) + abs(Rg1_y - s1_y);
	int Rg2_t = S1_t + abs(Rg2_x - s1_x) + abs(Rg2_y - s1_y);

	if (asymmetry_constraint) {
		constraints1.emplace_back(R1_x * num_col + R1_y, Rg1_x * num_col + Rg1_y, Rg1_t, constraint_type::BARRIER);
		for (int i = -k; i <= k; i++) {
			if ((Rg_t + i) < 0)
				continue;
			constraints2.emplace_back(R2_x * num_col + R2_y, Rg2_x * num_col + Rg2_y, Rg2_t + i, constraint_type::BARRIER);

		}
	}
	else {
		for (int i = 0; i <= k; i++) {
			constraints1.emplace_back(R1_x * num_col + R1_y, Rg1_x * num_col + Rg1_y, Rg1_t + i, constraint_type::BARRIER);

			constraints2.emplace_back(R2_x * num_col + R2_y, Rg2_x * num_col + Rg2_y, Rg2_t + i, constraint_type::BARRIER);
		}
	}

	//exit(0);
}

// add a vertival modified barrier constraint
bool addModifiedVerticalBarrierConstraint(const std::vector<PathEntry>& path, int y,
	int Ri_x, int Rg_x, int Rg_t, int num_col,
	std::list<Constraint>& constraints)
{
	int sign = Ri_x < Rg_x ? 1 : -1;
	int Ri_t = Rg_t - abs(Ri_x - Rg_x);
	int t1 = -1;
	int t_min = std::max(Ri_t, 0);
	int t_max = std::min(Rg_t, (int)path.size() - 1);
	for (int t2 = t_min; t2 <= t_max; t2++)
	{
		int loc = (Ri_x + (t2 - Ri_t) * sign) * num_col + y;
		std::list<int>::const_iterator it = std::find(path[t2].locations.begin(), path[t2].locations.end(), loc);
		if (it == path[t2].locations.end() && t1 >= 0) // add constraints [t1, t2)
		{
			int loc1 = (Ri_x + (t1 - Ri_t) * sign) * num_col + y;
			int loc2 = (Ri_x + (t2 - 1 - Ri_t) * sign) * num_col + y;
			constraints.emplace_back(loc1, loc2, t2 - 1, constraint_type::BARRIER);
			t1 = -1;
			continue;
		}
		else if (it != path[t2].locations.end() && t1 < 0)
		{
			t1 = t2;
		}
		if (it != path[t2].locations.end() && t2 == t_max)
		{
			int loc1 = (Ri_x + (t1 - Ri_t) * sign) * num_col + y;
			constraints.emplace_back(loc1, loc, t2, constraint_type::BARRIER); // add constraints [t1, t2]
		}
	}
	if (constraints.empty())
	{
		// std::cout << "Fail to add modified barrier constraints!" << std::endl;
		return false;
	}
	else
		return true;
}

// add a vertival modified barrier constraint
bool addModifiedVerticalLongBarrierConstraint(const std::vector<PathEntry>& path, int y,
	int Ri_x, int Rg_x, int Rg_t, int num_col, int St,
	std::list<Constraint>& constraints, int k, MDDPath* kMDD)
{


	//std::cout << "vertical y:" << y<<" Rix:"<<Ri_x<<" Rgx:"<< Rg_x<<" t:"<<Rg_t << std::endl;

	//for (int i = 0; i < kMDD.size(); i++) {
	//	for (int l = 0; l < kMDD[i]->levels.size(); l++) {
	//		std::unordered_set<int>::iterator it;
	//		std::cout << "level " << l << ": ";
	//		for (it = kMDD[i]->levels[l].begin(); it != kMDD[i]->levels[l].end(); ++it) {
	//			std::cout << *it << "," << *it << " ";
	//		}
	//		std::cout << std::endl;
	//	}
	//}

	int sign = Ri_x < Rg_x ? 1 : -1;
	int Ri_t = Rg_t - abs(Ri_x - Rg_x);
	std::unordered_set<string> added;

	for (int t2 = Ri_t; t2 <= Rg_t; t2++)
	{
		int loc = (Ri_x + (t2 - Ri_t) * sign) * num_col + y;
		//std::cout << "target loc: " << loc / num_col << "," << loc % num_col << std::endl;
		for (int i = 0; i <= k; i++) {
			//std::cout << "add constraint on k= " << i << " t=" << t2 << ": ";
			if ((t2 + i < path.size())) {
				std::list<int>::const_iterator it = std::find(path[t2 + i].locations.begin(), path[t2 + i].locations.end(), loc);
				if (it != path[t2 + i].locations.end())
				{
					for (int consk = 0; consk <= k - i; consk++) {
						//if constraint is on k=0, add more time range constraint until t=t+k
						std::stringstream con;
						con << loc << t2 + i + consk;
						if (!added.count(con.str())) {

							constraints.emplace_back(loc, -1, t2 + i + consk, constraint_type::VERTEX); // add constraints [t1, t2]
							//std::cout << "self mdd loc: " << loc / num_col << "," << loc % num_col << " t: " << t2 << "|";
							added.insert(con.str());
						}
					}



				}
				//std::cout << std::endl;
			}

			if (kMDD==NULL||t2 - St + i >= kMDD->levels.size())
				continue;
			if ((kMDD)->levels[t2 - St + i].count(loc)) {
				for (int consk = 0; consk <= k - i; consk++) {
					//if constraint is on k=0, add more time range constraint until t=t+k
					std::stringstream con;
					con << loc << t2 + i + consk;
					if (!added.count(con.str())) {
						constraints.emplace_back(loc, -1, t2 + i + consk, constraint_type::VERTEX); // add constraints [t1, t2]
						//std::cout << "kmdd loc: " << loc / num_col << "," << loc % num_col << " t: " << t2 + i + consk << "|";
						added.insert(con.str());
					}

				}
			}

			//	}
			//}
			//std::cout << std::endl;
		}

	}
	if (constraints.empty())
	{
		// std::cout << "Fail to add modified barrier constraints!" << std::endl;
		return false;
	}
	else
		return true;
}


// add a horizontal modified barrier constraint
bool addModifiedHorizontalLongBarrierConstraint(const std::vector<PathEntry>& path, int x,
	int Ri_y, int Rg_y, int Rg_t, int num_col, int St,
	std::list<Constraint>& constraints, int k, MDDPath* kMDD)
{
	//std::cout << "Horizontal x:" << x << " Riy:" << Ri_y << "Rgy:" << Rg_y << " t:" << Rg_t << std::endl;

	//for (int i = 0; i < kMDD.size(); i++) {
	//	for (int l = 0; l < kMDD[i]->levels.size(); l++) {
	//		std::unordered_set<int>::iterator it;
	//		std::cout << "level " << l << ": ";
	//		for (it = kMDD[i]->levels[l].begin(); it != kMDD[i]->levels[l].end(); ++it) {
	//			std::cout << *it << "," << *it << " ";
	//		}
	//		std::cout << std::endl;
	//	}
	//}

	int sign = Ri_y < Rg_y ? 1 : -1;
	int Ri_t = Rg_t - abs(Ri_y - Rg_y);
	int t1 = -1;
	bool overallFound = false;
	std::unordered_set<string> added;
	for (int t2 = Ri_t; t2 <= Rg_t; t2++)
	{
		int loc = (Ri_y + (t2 - Ri_t) * sign) + x * num_col;
		//std::cout << "target loc: " << loc / num_col << "," << loc % num_col << std::endl;
		for (int i = 0; i <= k; i++) {
			//std::cout << "add constraint on k= "<<i << " t=" << t2 << ": ";
			if ((t2 + i < path.size())) {

				std::list<int>::const_iterator it = std::find(path[t2 + i].locations.begin(), path[t2 + i].locations.end(), loc);

				if (it != path[t2 + i].locations.end())
				{
					for (int consk = 0; consk <= k - i; consk++) {
						//if constraint is on k=0, add more time range constraint until t=t+k
						std::stringstream con;
						con << loc << t2 + i + consk;
						if (!added.count(con.str())) {

							constraints.emplace_back(loc, -1, t2 + i + consk, constraint_type::VERTEX); // add constraints [t1, t2]
							//std::cout << "self mdd loc: " << loc / num_col << "," << loc % num_col << " t: " << t2 << "|";
							added.insert(con.str());
						}
					}
				}
			//std::cout << std::endl;
			}


			//if (kMDD != NULL){
			//	//std::cout << "add constraint on k=" << i << " t=" << t2 << ": ";
			//	for (int mdd = 0; mdd < (*kMDD).size(); mdd++) {
			//		if ((t2 - St + i) >= (*kMDD)[mdd]->levels.size())
			//			continue;
			if (kMDD == NULL||t2 - St + i >= kMDD->levels.size())
				continue;
			if (kMDD->levels[t2 - St + i].count(loc)) {
				for (int consk = 0; consk <= k - i; consk++) {
					//if constraint is on k=0, add more time range constraint until t=t+k
					std::stringstream con;
					con << loc << t2 + i + consk;
					if (!added.count(con.str())) {
						constraints.emplace_back(loc, -1, t2 + i + consk, constraint_type::VERTEX); // add constraints [t1, t2]
						//std::cout << "kmdd loc: " << loc / num_col << "," << loc % num_col << " t: " << t2 + i + consk << "|";
						added.insert(con.str());
					}

				}
			}


			//	}

			//}
			
			//std::cout << std::endl;
		}
	}
	if (constraints.empty())
	{
		// std::cout << "Fail to add modified barrier constraints!" << std::endl;
		return false;
	}
	else
		return true;


}

	// add a vertival modified barrier constraint
bool addFlippedVerticalLongBarrierConstraint(const std::vector<PathEntry>& path, int y,
	vector<int> vertical,vector<int> verticalMin,vector<int> verticalMax, int num_col, int St,
	std::list<Constraint>& constraints, int k, MDDPath* kMDD)
	{
		//std::cout << "vertical y:" << y<<" Rix:"<<Ri_x<<" Rgx:"<< Rg_x<<" t:"<<Rg_t << std::endl;

		std::unordered_set<string> added;

		for (int count = 0;count<vertical.size();count++)
		{
			int loc = vertical[count] * num_col + y;
			int tMin = verticalMin[count];
			int tMax = verticalMax[count];
			if (tMin > tMax)
				continue;
			//std::cout << "target loc: " << loc / num_col << "," << loc % num_col << std::endl;
			for (int t = tMin; t <= tMax; t++) {
				//std::cout << "add constraint on t=" << t << ": ";
				if ((t < path.size())) {
					std::list<int>::const_iterator it = std::find(path[t].locations.begin(), path[t].locations.end(), loc);
					if (it != path[t].locations.end())
					{
						std::stringstream con;
						con << loc << t;
						if (!added.count(con.str())) {
							//constraints.emplace_back(loc, -1, t, constraint_type::VERTEX); // add constraints [t1, t2]

							if(t==tMax)
								constraints.emplace_back(loc, -1, t, constraint_type::VERTEX); // add constraints [t1, t2]
							else
								constraints.emplace_back(loc, t, tMax, constraint_type::RANGE); // add constraints [t1, t2]
							
																								  //std::cout << "self mdd loc: " << loc / num_col << "," << loc % num_col << " t: " << t << "|";
							added.insert(con.str());
							break;
						}
					}
				}
				//else {
				//	std::cout << "t: " << t << " > " << path.size();
				//}
				//std::cout << std::endl;

				if (kMDD == NULL || t >= kMDD->levels.size())
					continue;
				if ((kMDD)->levels[t].count(loc)) {
					std::stringstream con;
					con << loc << t;
					if (!added.count(con.str())) {
						//constraints.emplace_back(loc, -1, t, constraint_type::VERTEX); // add constraints [t1, t2]

						if (t == tMax)
							constraints.emplace_back(loc, -1, t, constraint_type::VERTEX); // add constraints [t1, t2]
						else
							constraints.emplace_back(loc, t, tMax, constraint_type::RANGE); // add constraints [t1, t2]
						////std::cout << "kmdd loc: " << loc / num_col << "," << loc % num_col << " t: " << t2 + i + consk << "|";
						added.insert(con.str());
						break;
					}
				}
			}

		}
		if (constraints.empty())
		{
			// std::cout << "Fail to add modified barrier constraints!" << std::endl;
			return false;
		}
		else
			return true;
	}


	// add a horizontal modified barrier constraint
bool addFlippedHorizontalLongBarrierConstraint(const std::vector<PathEntry>& path, int x,
	vector<int> horizontal, vector<int> horizontalMin, vector<int> horizontalMax, int num_col, int St,
	std::list<Constraint>& constraints, int k, MDDPath* kMDD)
	{
		//std::cout << "Horizontal x:" << x << " Riy:" << Ri_y << "Rgy:" << Rg_y << " t:" << Rg_t << std::endl;

		std::unordered_set<string> added;
		for (int count = 0; count < horizontal.size(); count++)
		{
			int loc = x*num_col + horizontal[count] ;
			int tMin = horizontalMin[count];
			int tMax = horizontalMax[count];
			if (tMin > tMax)
				continue;
			//std::cout << "target loc: " << loc / num_col << "," << loc % num_col << std::endl;
			for (int t = tMin; t <= tMax; t++) {
				//std::cout << "add constraint on t=" << t << ": ";
				if (t < path.size()) {
					std::list<int>::const_iterator it = std::find(path[t].locations.begin(), path[t].locations.end(), loc);
					if (it != path[t].locations.end())
					{
						std::stringstream con;
						con << loc << t;
						if (!added.count(con.str())) {
							//constraints.emplace_back(loc, -1, t, constraint_type::VERTEX); // add constraints [t1, t2]

							if (t == tMax)
								constraints.emplace_back(loc, -1, t, constraint_type::VERTEX); // add constraints [t1, t2]
							else
								constraints.emplace_back(loc, t, tMax, constraint_type::RANGE); // add constraints [t1, t2]
							//std::cout << "self mdd loc: " << loc / num_col << "," << loc % num_col << " t: " << t << "|";
							added.insert(con.str());
							break;
						}
					}
					
				}
				//else {
				//	std::cout << "t: " << t << " > " << path.size();
				//}
				//std::cout << std::endl;

				if (kMDD == NULL || t >= kMDD->levels.size())
					continue;
				if ((kMDD)->levels[t].count(loc)) {
					std::stringstream con;
					con << loc << t;
					if (!added.count(con.str())) {
						//constraints.emplace_back(loc, -1, t, constraint_type::VERTEX); // add constraints [t1, t2]

						if (t == tMax)
							constraints.emplace_back(loc, -1, t, constraint_type::VERTEX); // add constraints [t1, t2]
						else
							constraints.emplace_back(loc, t, tMax, constraint_type::RANGE); // add constraints [t1, t2]
						////std::cout << "kmdd loc: " << loc / num_col << "," << loc % num_col << " t: " << t2 + i + consk << "|";
						added.insert(con.str());
						break;
					}
				}
			}
		}
		if (constraints.empty())
		{
			// std::cout << "Fail to add modified barrier constraints!" << std::endl;
			return false;
		}
		else
			return true;


	}
*/