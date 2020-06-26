#pragma once


#include "LLNode.h"
#include <memory>
#include "common.h"
#include <string>
#include <sstream>
using namespace std;




// <loc, -1, t, VERTEX>
// <from, to, t, EDGE> 
// <B1, B2, t, RECTANGLE>
// <loc, t1, t2, CORRIDOR> 
// <loc, agent_id, t, TARGET>: path of agent_id should be of length at most t, and any other agent cannot be at loc at or after timestep t
// <-1, agent_id, t>: path of agent_id should be of length at least t + 1 


std::ostream& operator<<(std::ostream& os, const Constraint& constraint);


// add a pair of barrier constraints
void addBarrierConstraints(int S1, int S2, int S1_t, int S2_t, int Rg, int num_col,
	std::list<std::tuple<int, int, int>>& constraints1, std::list<std::tuple<int, int, int>>& constraints2);

// add a pair of modified barrier constraints
bool addModifiedBarrierConstraints(const std::vector<PathEntry>& path1, const std::vector<PathEntry>& path2,
	int S1_t, int S2_t, int Rg, int num_col,
	std::list<std::tuple<int, int, int>>& constraints1, std::list<std::tuple<int, int, int>>& constraints2);

// add a horizontal modified barrier constraint
bool addModifiedHorizontalBarrierConstraint(const std::vector<PathEntry>& path, int x,
	int Ri_y, int Rg_y, int Rg_t, int num_col,
	std::list<Constraint>& constraints);

// add a vertival modified barrier constraint
bool addModifiedVerticalBarrierConstraint(const std::vector<PathEntry>& path, int y,
	int Ri_x, int Rg_x, int Rg_t, int num_col,
	std::list<Constraint>& constraints);

void addKDelayBarrierConstraints(int S1, int S2, int S1_t, int S2_t, int Rg, int G1, int G2, int num_col,
	std::list<Constraint>& constraints1, std::list<Constraint>& constraints2,
	int k, bool asymmetry_constraint);

// add a vertival modified barrier constraint
bool addModifiedVerticalLongBarrierConstraint(const std::vector<PathEntry>& path, int y,
	int Ri_x, int Rg_x, int Rg_t, int num_col, int St,
	std::list<Constraint>& constraints, int k, MDDPath* kMDD);

// add a horizontal modified barrier constraint
bool addModifiedHorizontalLongBarrierConstraint(const std::vector<PathEntry>& path, int x,
	int Ri_y, int Rg_y, int Rg_t, int num_col, int St,
	std::list<Constraint>& constraints, int k, MDDPath* kMDD);

bool addFlippedVerticalLongBarrierConstraint(const std::vector<PathEntry>& path, int y,
	vector<int> vertical, vector<int> verticalMin, vector<int> verticalMax, int num_col, int St,
	std::list<Constraint>& constraints, int k, MDDPath* kMDD);

bool addFlippedHorizontalLongBarrierConstraint(const std::vector<PathEntry>& path, int x,
	vector<int> horizontal, vector<int> horizontalMin, vector<int> horizontalMax, int num_col, int St,
	std::list<Constraint>& constraints, int k, MDDPath* kMDD);




class Conflict
{
public:

	int a1;
	int a2;
	int t;
	int k=0;
	int s1;
	int s2;
	int g1;
	int g2;
	int s1_t;
	int s2_t;
	int g1_t;
	int g2_t;
	int rs=0;
	int rg=0;
	int t_sg;
	int originalConf1=0;
	int originalConf2=-1;
	int flipType = 0;
	bool repeat = false;
	std::list<Constraint> constraint1;
	std::list<Constraint> constraint2;
	conflict_type type;
	conflict_priority p = conflict_priority::UNKNOWN;

	Conflict() {};
	Conflict(int v,int t) {
		this->originalConf1 = v;
		this->originalConf2 = -1;
		this->t = t;
	};

	void vertexConflict(int a1, int a2, int v, int t,int k=0,int kRobust =0)
	{
		this->a1 = a1;
		this->a2 = a2;
		this->t = t;
		this->k = k;
		this->originalConf1 = v;
		this->originalConf2 = -1;
		for (int i = 0; i <= kRobust; i++) {
			this->constraint1.emplace_back(v, -1, t+i, constraint_type::VERTEX);
			this->constraint2.emplace_back(v, -1, t+i, constraint_type::VERTEX);
		}
		type = conflict_type::STANDARD;
	}
		
	void edgeConflict(int a1, int a2, int v1, int v2, int t)
	{
		this->a1 = a1;
		this->a2 = a2;
		this->t = t;
		this->k = 0;
		this->originalConf1 = v1;
		this->originalConf2 = v2;

		this->constraint1.emplace_back(v1, v2, t, constraint_type::EDGE);
		this->constraint2.emplace_back(v2, v1, t, constraint_type::EDGE);
		type = conflict_type::STANDARD;
	}

	void trainCorridorConflict(int a1, int a2, int v1, int v2, int t1, int t2, int e1, int e2, int k, int kRobust)
	{
		this->a1 = a1;
		this->a2 = a2;
		this->k = k;
		this->t = std::min(e1, e2);
		this->originalConf1 = v1;
		this->originalConf2 = v2;
		this->constraint1.emplace_back(v1, t2, e2-1 + kRobust, constraint_type::RANGE);
		this->constraint2.emplace_back(v2, t1, e1-1 + kRobust, constraint_type::RANGE);
		type = conflict_type::CORRIDOR2;
	}

	// t3 
	void corridorConflict(int a1, int a2, int v1, int v2, int t3, int t4, int t3_, int t4_, int k,int kRobust)
	{
		this->a1 = a1;
		this->a2 = a2;
		this->k = k;
		this->t = t3*1000+t4;
		this->originalConf1 = v1;
		this->originalConf2 = v2;
		if(t3<=std::min(t3_ - 1, t4 + k) + kRobust)
			this->constraint1.emplace_back(v1, t3, std::min(t3_ - 1 , t4 + k) + kRobust, constraint_type::RANGE);
		if(t4<= std::min(t4_ - 1, t3 + k) + kRobust)
			this->constraint2.emplace_back(v2, t4, std::min(t4_ - 1 , t3 + k) + kRobust, constraint_type::RANGE);
		type = conflict_type::CORRIDOR2;
	}

	void chasingConflict(int a1,int a2,int v1, int v2,int exit, int early,int late, int kRobust)
	{
		this->a1 = a1;
		this->a2 = a2;
		this->k = kRobust;
		this->t =  early * 1000 + late;
		this->originalConf1 = v1;
		this->originalConf2 = v2;
		
		this->constraint1.emplace_back(exit, early, late+ kRobust, constraint_type::RANGE);
		type = conflict_type::CORRIDOR2;
	}


	void corridorConflict(int a1, int a2, int v1, int v2, int t1, int t2, int k, int h, int kRobust)
	{
		this->a1 = a1;
		this->a2 = a2;
		this->t = std::min(t1, t2);
		this->originalConf1 = v1;
		this->originalConf2 = v2;
		this->k = k;
		this->constraint1.emplace_back(v1, t1, t1 + 2 * k - 1 + kRobust, constraint_type::RANGE);
		this->constraint1.emplace_back(v2, t1 + k, std::min(t2 + 2 * k, t1 + h - 1) + kRobust, constraint_type::RANGE);
		this->constraint2.emplace_back(v2, t2, t2 + 2 * k - 1 + kRobust, constraint_type::RANGE);
		this->constraint2.emplace_back(v1, t2 + k, std::min(t1 + 2 * k, t2 + h - 1) + kRobust, constraint_type::RANGE);
		type = conflict_type::CORRIDOR4;
	}

	bool rectangleConflict(int a1, int a2, const std::pair<int, int>& Rs, const std::pair<int, int>& Rg, int move1, int move2, int Rg_t, const std::vector<Path*>& paths, int num_col) // For GR
	{
		this->a1 = a1;
		this->a2 = a2;
		this->k = 0;
		this->t_sg = Rg_t - abs(Rg.first - Rs.first) - abs(Rg.second - Rs.second);
		this->rs = Rs.first*num_col + Rs.second;
		this->rg = Rg.first*num_col + Rg.second;;
		if (abs(move1) == 1 || abs(move2) > 1) // first agent moves horizontally and second agent moves vertically
		{
			if (!addModifiedVerticalBarrierConstraint(*paths[a1], Rg.second, Rs.first, Rg.first, Rg_t, num_col, constraint1))
			{
				return false;
			}
			if (!addModifiedHorizontalBarrierConstraint(*paths[a2], Rg.first, Rs.second, Rg.second, Rg_t, num_col, constraint2))
			{
				return false;
			}
		}
		else
		{
			if (!addModifiedHorizontalBarrierConstraint(*paths[a1], Rg.first, Rs.second, Rg.second, Rg_t, num_col, constraint1))
			{
				return false;
			}
			if (!addModifiedVerticalBarrierConstraint(*paths[a2], Rg.second, Rs.first, Rg.first, Rg_t, num_col, constraint2))
			{
				return false;
			}
		}
		type = conflict_type::RECTANGLE;
		return true;
	}

	bool rectangleConflict(int a1, int a2, const std::pair<int, int>& Rs, const std::pair<int, int>& Rg,
		const std::pair<int, int>& s1, const std::pair<int, int>& s2, int Rg_t, const std::vector<Path*>& paths, int num_col) // For RM
	{
		this->a1 = a1;
		this->a2 = a2;
		this->k = 0;
		this->t_sg = Rg_t - abs(Rg.first - Rs.first) - abs(Rg.second - Rs.second);
		this->rs = Rs.first*num_col + Rs.second;
		this->rg = Rg.first*num_col + Rg.second;;
		if (s1.first == s2.first)
		{
			if ((s1.second - s2.second) * (s2.second - Rg.second) >= 0)
			{
				// first agent moves horizontally and second agent moves vertically
				if (!addModifiedVerticalBarrierConstraint(*paths[a1], Rg.second, Rs.first, Rg.first, Rg_t, num_col, constraint1))
				{
					return false;
				}
				if (!addModifiedHorizontalBarrierConstraint(*paths[a2], Rg.first, Rs.second, Rg.second, Rg_t, num_col, constraint2))
				{
					return false;
				}
			}
			else
			{
				// first agent moves vertically and second agent moves horizontally
				if (!addModifiedHorizontalBarrierConstraint(*paths[a1], Rg.first, Rs.second, Rg.second, Rg_t, num_col, constraint1))
				{
					return false;
				}
				if (!addModifiedVerticalBarrierConstraint(*paths[a2], Rg.second, Rs.first, Rg.first, Rg_t, num_col, constraint2))
				{
					return false;
				}
			}
		}
		else if ((s1.first - s2.first)*(s2.first - Rg.first) >= 0)
		{
			// first agent moves vertically and second agent moves horizontally
			if (!addModifiedHorizontalBarrierConstraint(*paths[a1], Rg.first, Rs.second, Rg.second, Rg_t, num_col, constraint1))
			{
				return false;
			}
			if (!addModifiedVerticalBarrierConstraint(*paths[a2], Rg.second, Rs.first, Rg.first, Rg_t, num_col, constraint2))
			{
				return false;
			}
		}
		else
		{
			// first agent moves horizontally and second agent moves vertically
			if (!addModifiedVerticalBarrierConstraint(*paths[a1], Rg.second, Rs.first, Rg.first, Rg_t, num_col, constraint1))
			{
				return false;
			}
			if (!addModifiedHorizontalBarrierConstraint(*paths[a2], Rg.first, Rs.second, Rg.second, Rg_t, num_col, constraint2))
			{
				return false;
			}
		}
		type = conflict_type::RECTANGLE;
		return true;
	}


	bool kRectangleConflict(int a1, int a2, int S1, int S2, int S1_t, int S2_t, pair<int, int> Rs, pair<int, int> Rg,
		int Rg_t, int G1, int G2, int num_col,int k,bool asymmetry_constraint) // For CR and R
	{
		this->a1 = a1;
		this->a2 = a2;
		this->t_sg = Rg_t - abs(Rg.first - Rs.first) - abs(Rg.second - Rs.second);
		this->rs = Rs.first*num_col + Rs.second;
		this->rg = Rg.first*num_col + Rg.second;
		this->k = k;
		addKDelayBarrierConstraints(S1, S2, S1_t, S2_t, Rg.first*num_col+Rg.second, G1, G2, num_col,
			constraint1, constraint2,k, asymmetry_constraint);

		type = conflict_type::RECTANGLE;
		return true;
	}

	bool kRectangleConflict(int a1, int a2, const std::pair<int, int>& Rs, const std::pair<int, int>& Rg,
		const std::pair<int, int>& s1, const std::pair<int, int>& s2, int Rg_t,
		const std::vector<Path*>& paths, int S1_t,int S2_t, const std::pair<int, int>& G1, const std::pair<int, int>& G2,
		int num_col, int k, MDDPath* a1kMDD=NULL, MDDPath* a2kMDD=NULL ) // For K-RM
	{
		this->a1 = a1;
		this->a2 = a2;
		this->t_sg = Rg_t - abs(Rg.first - Rs.first) - abs(Rg.second - Rs.second);
		this->rs = Rs.first*num_col + Rs.second;
		this->rg = Rg.first*num_col + Rg.second;
		this->k = k;

		int s1_x = s1.first;
		int s1_y = s1.second;
		int s2_x = s2.first;
		int s2_y = s2.second;
		int Rg_x = Rg.first;
		int Rg_y = Rg.second;
		int g1_x = G1.first;
		int g1_y = G1.second;
		int g2_x = G2.first;
		int g2_y = G2.second;


		int R1_x, R1_y, R2_x, R2_y, G1_x, G1_y, G2_x, G2_y, G1_t, G2_t;
		if ((s1_x == s2_x && (s1_y - s2_y) * (s2_y - Rg_y) < 0) ||
			(s1_x != s2_x && (s1_x - s2_x)*(s2_x - Rg_x) >= 0))
		{
			R1_x = Rg_x;
			G1_x = Rg_x;

			R2_x = s1_x;
			G2_x = g1_x;

			R1_y = s2_y;
			G1_y = g2_y;

			R2_y = Rg_y;
			G2_y = Rg_y;

			G1_t = Rg_t + abs(G1_y - Rg_y);
			G2_t = Rg_t + abs(G2_x - Rg_x);
			//cout << "s1t" << S1_t << "G1_t" << G1_t << "G1_y " << G1_y << endl;
			//cout << "s2t" << S2_t << "G2_t" << G2_t << "G2_x " << G2_x << endl;

			
			if (!addModifiedHorizontalLongBarrierConstraint(*paths[a1], Rg_x, R1_y, G1_y, G1_t, num_col, S1_t, constraint1, k, a1kMDD))
				return false;
			if (!addModifiedVerticalLongBarrierConstraint(*paths[a2], Rg_y, R2_x, G2_x, G2_t, num_col, S2_t, constraint2, k, a2kMDD))
				return false;
		}
		else
		{
			R1_x = s2_x;
			G1_x = g2_x;

			R2_x = Rg_x;
			G2_y = Rg_x;

			R1_y = Rg_y;
			G1_y = Rg_y;

			R2_y = s1_y;
			G2_y = g1_y;


			G1_t = Rg_t + abs(G1_x - Rg_x);
			G2_t = Rg_t + abs(G2_y - Rg_y);
			//cout << "s1t" << S1_t << "G1_t" << G1_t << "G1_y " << G1_y << endl;
			//cout << "s2t" << S2_t << "G2_t" << G2_t << "G2_x " << G2_x << endl;
			
			if(!addModifiedVerticalLongBarrierConstraint(*paths[a1], Rg_y, R1_x, G1_x, G1_t, num_col, S1_t, constraint1, k, a1kMDD))
				return false;
			if (!addModifiedHorizontalLongBarrierConstraint(*paths[a2], Rg_x, R2_y, G2_y, G2_t, num_col, S2_t, constraint2, k, a2kMDD))
				return false;
			//exit(0);
		}
		type = conflict_type::RECTANGLE;
		return true;
	}

	bool flippedRectangleConflict(int a1, int a2, const std::pair<int, int>& Rs, const std::pair<int, int>& Rg,
		const std::pair<int, int>& s1, const std::pair<int, int>& s2, int Rg_t,
		const std::vector<Path*>& paths, int S1_t, int S2_t, const std::pair<int, int>& G1, const std::pair<int, int>& G2,
		int num_col, int k,int flipType, MDDPath* a1kMDD = NULL, MDDPath* a2kMDD = NULL) // For K-RM
	{
		this->a1 = a1;
		this->a2 = a2;
		this->t_sg = Rg_t - abs(Rg.first - Rs.first) - abs(Rg.second - Rs.second);
		this->rs = Rs.first*num_col + Rs.second;
		this->rg = Rg.first*num_col + Rg.second;
		this->k = k;
		this->flipType = flipType;

		int s1_x = s1.first;
		int s1_y = s1.second;
		int s2_x = s2.first;
		int s2_y = s2.second;
		int Rg_x = Rg.first;
		int Rg_y = Rg.second;
		int g1_x = G1.first;
		int g1_y = G1.second;
		int g2_x = G2.first;
		int g2_y = G2.second;


		int R1_x, R1_y, R2_x, R2_y, G1_x, G1_y, G2_x, G2_y;
		vector<int> horizontal;
		vector<int> horizontalMin;
		vector<int> horizontalMax;

		vector<int> vertical;
		vector<int> verticalMin;
		vector<int> verticalMax;

		if (flipType == 2) {
			if ((g2_x == s1_x && (g1_y - s1_y) * (s1_y - Rg_y) >= 0) ||
				(g2_x != s1_x && (g2_x - s1_x) * (s1_x - Rg_x) < 0)) {

				G1_x = Rg.first;
				G1_y = g2_y;
				R1_y = s2_y;

				int sign = R1_y < G1_y ? 1 : -1;
				horizontal.clear();
				horizontalMin.clear();
				horizontalMin.clear();
				for (int y = G1_y; y != R1_y - sign; y = y - sign * 1) {
					horizontal.push_back(y);
					horizontalMin.push_back(getMahattanDistance(s1_x, s1_y, G1_x, y) + S1_t);
					horizontalMax.push_back(getMahattanDistance(s2_x, s2_y, G1_x, y) + k + S2_t);
				}

				if (!addFlippedHorizontalLongBarrierConstraint(*paths[a1], G1_x, horizontal, horizontalMin, horizontalMax, num_col, S1_t, constraint1, k, a1kMDD))
					return false;

				G2_y = Rs.second;
				G2_x = g1_x;
				R2_x = s1_x;

				sign = R2_x < G2_x ? 1 : -1;
				vertical.clear();
				verticalMin.clear();
				verticalMax.clear();
				for (int x = G2_x; x != R2_x - sign; x = x - sign * 1) {
					vertical.push_back(x);
					verticalMin.push_back(getMahattanDistance(s2_x, s2_y, x, G2_y) + S2_t);
					verticalMax.push_back(getMahattanDistance(s1_x, s1_y, x, G2_y) + k + S1_t);
				}

				if (!addFlippedVerticalLongBarrierConstraint(*paths[a2], G2_y, vertical, verticalMin, verticalMax, num_col, S2_t, constraint2, k, a2kMDD))
					return false;

			
			}
			else {
				G1_y = Rg.second;
				G1_x = g2_x;
				R1_x = s2_x;

				int sign = R1_x < G1_x ? 1 : -1;
				vertical.clear();
				verticalMin.clear();
				verticalMax.clear();
				for (int x = G1_x; x != R1_x - sign; x = x - sign * 1) {
					vertical.push_back(x);
					verticalMin.push_back(getMahattanDistance(s1_x, s1_y, x, G1_y) + S1_t);
					verticalMax.push_back(getMahattanDistance(s2_x, s2_y, x, G1_y) + k + S2_t);
				}

				if (!addFlippedVerticalLongBarrierConstraint(*paths[a1], G1_y, vertical, verticalMin, verticalMax, num_col, S1_t, constraint1, k, a1kMDD))
					return false;

				G2_x = Rs.first;
				G2_y = g1_y;
				R2_y = s1_y;

				sign = R2_y < G2_y ? 1 : -1;
				horizontal.clear();
				horizontalMin.clear();
				horizontalMin.clear();
				for (int y = G2_y; y != R2_y - sign; y = y - sign * 1) {
					horizontal.push_back(y);
					horizontalMin.push_back(getMahattanDistance(s2_x, s2_y, G2_x, y) + S2_t);
					horizontalMax.push_back(getMahattanDistance(s1_x, s1_y, G2_x, y) + k + S1_t);
				}

				if (!addFlippedHorizontalLongBarrierConstraint(*paths[a2], G2_x, horizontal, horizontalMin, horizontalMax, num_col, S2_t, constraint2, k, a2kMDD))
					return false;

			}
			




		}
		else if (flipType == 1) {//1 flipped case
			if ((s1_y - g1_y) * (s2_y - g2_y) < 0) {// y dimension flipped
				if (Rs.second != g1_y) {//s1 vertical border need constraint
					G1_y = Rs.second;
					G1_x = g2_x;
					R1_x = s2_x;

					int sign = R1_x < G1_x ? 1 : -1;
					vertical.clear();
					verticalMin.clear();
					verticalMax.clear();
					for (int x = G1_x; x != R1_x-sign; x = x - sign * 1) {
						vertical.push_back(x);
						//int tMin = min(getMahattanDistance(s1_x, s1_y, x, G1_y) + S1_t, getMahattanDistance(s2_x, s2_y, x, G1_y) + S2_t);
						//verticalMin.push_back(tMin);
						//verticalMax.push_back(tMin+k);
						verticalMin.push_back(getMahattanDistance(s1_x, s1_y, x, G1_y) + S1_t);
						verticalMax.push_back(getMahattanDistance(s2_x, s2_y, x, G1_y) + k + S2_t);
					}

					if (!addFlippedVerticalLongBarrierConstraint(*paths[a1], G1_y, vertical, verticalMin, verticalMax, num_col, S1_t, constraint1, k, a1kMDD))
						return false;

				}

				if (Rg.first != g1_x || (Rs.second == g1_y && Rg.first == g1_x)) {//s1 horizontoal border need constraint
					G1_x = Rg.first;
					G1_y = g2_y;
					R1_y = s2_y;

					int sign = R1_y < G1_y ? 1 : -1;
					horizontal.clear();
					horizontalMin.clear();
					horizontalMin.clear();
					for (int y = G1_y; y != R1_y-sign; y = y - sign * 1) {
						horizontal.push_back(y);
						//int tMin = min(getMahattanDistance(s1_x, s1_y, G1_x, y) + S1_t, getMahattanDistance(s2_x, s2_y, G1_x, y) + S2_t);
						//horizontalMin.push_back(tMin);
						//horizontalMax.push_back(tMin + k);
						horizontalMin.push_back(getMahattanDistance(s1_x, s1_y, G1_x, y) + S1_t);
						horizontalMax.push_back(getMahattanDistance(s2_x, s2_y, G1_x, y) + k + S2_t);
					}

					if (!addFlippedHorizontalLongBarrierConstraint(*paths[a1], G1_x, horizontal, horizontalMin, horizontalMax, num_col, S1_t, constraint1, k, a1kMDD))
						return false;

				}

				if (Rg.second != g2_y ) {//s2 vertical border need constraint
					G2_y = Rg.second;
					G2_x = g1_x;
					R2_x = s1_x;

					int sign = R2_x < G2_x ? 1 : -1;
					vertical.clear();
					verticalMin.clear();
					verticalMax.clear();
					for (int x = G2_x; x != R2_x - sign; x = x - sign * 1) {
						vertical.push_back(x);
						/*int tMin = min(getMahattanDistance(s1_x, s1_y, x, G2_y) + S1_t, getMahattanDistance(s2_x, s2_y, x, G2_y) + S2_t);
						verticalMin.push_back(tMin);
						verticalMax.push_back(tMin + k);*/
						verticalMin.push_back(getMahattanDistance(s2_x, s2_y, x, G2_y) + S2_t);
						verticalMax.push_back(getMahattanDistance(s1_x, s1_y, x, G2_y) + k + S1_t);
					}

					if (!addFlippedVerticalLongBarrierConstraint(*paths[a2], G2_y, vertical, verticalMin, verticalMax, num_col, S2_t, constraint2, k, a2kMDD))
						return false;

				}

				if (Rg.first != g2_x || (Rg.second == g2_y && Rg.first == g2_x)) {//s2 horizontoal border need constraint
					G2_x = Rg.first;
					G2_y = g1_y;
					R2_y = s1_y;

					int sign = R2_y < G2_y ? 1 : -1;
					horizontal.clear();
					horizontalMin.clear();
					horizontalMin.clear();
					for (int y = G2_y; y != R2_y - sign; y = y - sign * 1) {
						horizontal.push_back(y);
						/*int tMin = min(getMahattanDistance(s1_x, s1_y, G2_x, y) + S1_t, getMahattanDistance(s2_x, s2_y, G2_x, y) + S2_t);
						horizontalMin.push_back(tMin);
						horizontalMax.push_back(tMin + k);*/
						horizontalMin.push_back(getMahattanDistance(s2_x, s2_y, G2_x, y) + S2_t);
						horizontalMax.push_back(getMahattanDistance(s1_x, s1_y, G2_x, y) + k + S1_t);
					}

					if (!addFlippedHorizontalLongBarrierConstraint(*paths[a2], G2_x, horizontal, horizontalMin, horizontalMax, num_col, S2_t, constraint2, k, a2kMDD))
						return false;

				}


			}
			else {// x dimension flipped
				if (Rg.second != g1_y || (Rg.second == g1_y && Rs.first == g1_x)) {//s1 vertical border need constraint
					G1_y = Rg.second;
					G1_x = g2_x;
					R1_x = s2_x;

					int sign = R1_x < G1_x ? 1 : -1;
					vertical.clear();
					verticalMin.clear();
					verticalMax.clear();
					for (int x = G1_x; x != R1_x-sign; x = x - sign * 1) {
						vertical.push_back(x);
						/*int tMin = min(getMahattanDistance(s1_x, s1_y, x, G1_y) + S1_t, getMahattanDistance(s2_x, s2_y, x, G1_y) + S2_t);
						verticalMin.push_back(tMin);
						verticalMax.push_back(tMin + k);*/
						verticalMin.push_back(getMahattanDistance(s1_x, s1_y, x, G1_y) + S1_t);
						verticalMax.push_back(getMahattanDistance(s2_x, s2_y, x, G1_y) + k + S2_t);
					}

					if (!addFlippedVerticalLongBarrierConstraint(*paths[a1], G1_y, vertical, verticalMin, verticalMax, num_col, S1_t, constraint1, k, a1kMDD))
						return false;

				}

				if (Rs.first != g1_x) {//s1 horizontoal border need constraint
					G1_x = Rs.first;
					G1_y = g2_y;
					R1_y = s2_y;

					int sign = R1_y < G1_y ? 1 : -1;
					horizontal.clear();
					horizontalMin.clear();
					horizontalMin.clear();
					for (int y = G1_y; y != R1_y-sign; y = y - sign * 1) {
						horizontal.push_back(y);
						/*int tMin = min(getMahattanDistance(s1_x, s1_y, G1_x, y) + S1_t, getMahattanDistance(s2_x, s2_y, G1_x, y) + S2_t);
						horizontalMin.push_back(tMin);
						horizontalMax.push_back(tMin + k);*/
						horizontalMin.push_back(getMahattanDistance(s1_x, s1_y, G1_x, y) + S1_t);
						horizontalMax.push_back(getMahattanDistance(s2_x, s2_y, G1_x, y) + k + S2_t);
					}

					if (!addFlippedHorizontalLongBarrierConstraint(*paths[a1], G1_x, horizontal, horizontalMin, horizontalMax, num_col, S1_t, constraint1, k, a1kMDD))
						return false;

				}

				if (Rg.second != g2_y || (Rg.second == g2_y && Rg.first == g2_x)) {//s2 vertical border need constraint
					G2_y = Rg.second;
					G2_x = g1_x;
					R2_x = s1_x;

					int sign = R2_x < G2_x ? 1 : -1;
					vertical.clear();
					verticalMin.clear();
					verticalMax.clear();
					for (int x = G2_x; x != R2_x - sign; x = x - sign * 1) {
						vertical.push_back(x);
						/*int tMin = min(getMahattanDistance(s1_x, s1_y, x, G2_y) + S1_t, getMahattanDistance(s2_x, s2_y, x, G2_y) + S2_t);
						verticalMin.push_back(tMin);
						verticalMax.push_back(tMin + k);*/
						verticalMin.push_back(getMahattanDistance(s2_x, s2_y, x, G2_y) + S2_t);
						verticalMax.push_back(getMahattanDistance(s1_x, s1_y, x, G2_y) + k + S1_t);
					}

					if (!addFlippedVerticalLongBarrierConstraint(*paths[a2], G2_y, vertical, verticalMin, verticalMax, num_col, S2_t, constraint2, k, a2kMDD))
						return false;

				}

				if (Rg.first != g2_x) {//s2 horizontoal border need constraint
					G2_x = Rg.first;
					G2_y = g1_y;
					R2_y = s1_y;

					int sign = R2_y < G2_y ? 1 : -1;
					horizontal.clear();
					horizontalMin.clear();
					horizontalMin.clear();
					for (int y = G2_y; y != R2_y - sign; y = y - sign * 1) {
						horizontal.push_back(y);
						/*int tMin = min(getMahattanDistance(s1_x, s1_y, G2_x, y) + S1_t, getMahattanDistance(s2_x, s2_y, G2_x, y) + S2_t);
						horizontalMin.push_back(tMin);
						horizontalMax.push_back(tMin + k);*/
						horizontalMin.push_back(getMahattanDistance(s2_x, s2_y, G2_x, y) + S2_t);
						horizontalMax.push_back(getMahattanDistance(s1_x, s1_y, G2_x, y) + k + S1_t);
					}

					if (!addFlippedHorizontalLongBarrierConstraint(*paths[a2], G2_x, horizontal, horizontalMin, horizontalMax, num_col, S2_t, constraint2, k, a2kMDD))
						return false;

				}
			}

			

		}
		else {//standard non flip
		
			if ((s1_x == s2_x && (s1_y - s2_y) * (s2_y - Rg_y) < 0) ||
				(s1_x != s2_x && (s1_x - s2_x)*(s2_x - Rg_x) >= 0))
			{
				R1_x = Rg_x;
				G1_x = Rg_x;
				R1_y = s2_y;
				G1_y = g2_y;

				R2_y = Rg_y;
				G2_y = Rg_y;
				R2_x = s1_x;
				G2_x = g1_x;
				

				//cout << "s1t" << S1_t << "G1_t" << G1_t << "G1_y " << G1_y << endl;
				//cout << "s2t" << S2_t << "G2_t" << G2_t << "G2_x " << G2_x << endl;
				
				int sign = R1_y < G1_y ? 1 : -1;
				for (int y = G1_y; y != R1_y-sign; y = y - sign * 1) {
					horizontal.push_back(y);
					horizontalMin.push_back(getMahattanDistance(s1_x, s1_y, Rg_x, y) + S1_t);
					horizontalMax.push_back(getMahattanDistance(s2_x, s2_y, Rg_x, y) + k + S2_t);
				}


				sign = R2_x < G2_x ? 1 : -1;
				for (int x = G2_x; x != R2_x-sign; x = x - sign * 1) {
					vertical.push_back(x);
					verticalMin.push_back(getMahattanDistance(s2_x, s2_y, x, Rg_y) + S2_t);
					verticalMax.push_back(getMahattanDistance(s1_x, s1_y, x, Rg_y) + k + S1_t);
				}

				//cout << "Horizontal length: " << horizontal.size() << " Vertical length: " << vertical.size() << endl;


				if (!addFlippedHorizontalLongBarrierConstraint(*paths[a1], Rg_x, horizontal, horizontalMin, horizontalMax, num_col, S1_t, constraint1, k, a1kMDD))
					return false;
				if (!addFlippedVerticalLongBarrierConstraint(*paths[a2], Rg_y, vertical, verticalMin, verticalMax, num_col, S2_t, constraint2, k, a2kMDD))
					return false;
			}
			else
			{
				R1_y = Rg_y;
				G1_y = Rg_y;
				R1_x = s2_x;
				G1_x = g2_x;
				

				R2_x = Rg_x;
				G2_x = Rg_x;
				R2_y = s1_y;
				G2_y = g1_y;
				

				


				int sign = R2_y < G2_y ? 1 : -1;

				for (int y = G2_y; y != R2_y-sign; y = y - sign * 1) {
					horizontal.push_back(y);
					horizontalMin.push_back(getMahattanDistance(s2_x, s2_y, Rg_x, y) + S2_t);
					horizontalMax.push_back(getMahattanDistance(s1_x, s1_y, Rg_x, y) + k + S1_t);
				}

				sign = R1_x < G1_x ? 1 : -1;
				for (int x = G1_x; x != R1_x-sign; x = x - sign * 1) {
					vertical.push_back(x);
					verticalMin.push_back(getMahattanDistance(s1_x, s1_y, x, Rg_y)+S1_t);
					verticalMax.push_back(getMahattanDistance(s2_x, s2_y, x, Rg_y) + k + S2_t);
				}


				if (!addFlippedVerticalLongBarrierConstraint(*paths[a1], Rg_y, vertical, verticalMin, verticalMax, num_col, S1_t, constraint1, k, a1kMDD))
					return false;
				if (!addFlippedHorizontalLongBarrierConstraint(*paths[a2], Rg_x, horizontal, horizontalMin, horizontalMax, num_col, S2_t, constraint2, k, a2kMDD))
					return false;
				//exit(0);
			}
		}
		
		type = conflict_type::RECTANGLE;
		return true;
	}

	void targetConflict(int a1, int a2, int v, int t,int kDelay)
	{
		this->a1 = a1;
		this->a2 = a2;
		this->t = t;
		this->k = 0;
		this->originalConf1 = -1;
		this->originalConf2 = a1;

		this->constraint1.emplace_back(-1, a1, t + kDelay/* kDelay>0? t + kDelay+1:t*/, constraint_type::LENGTH); // length of a1 should be larger than t
		this->constraint2.emplace_back(v, a1, t, constraint_type::LENGTH); // length of a1 should be no larger than t, and other agents can not use v at and after timestep t
		type = conflict_type::TARGET;
	}


};

std::ostream& operator<<(std::ostream& os, const Conflict& conflict);

bool operator < (const Conflict& conflict1, const Conflict& conflict2);





