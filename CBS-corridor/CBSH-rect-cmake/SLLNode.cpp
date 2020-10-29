#include "SLLNode.h"

SLLNode::SLLNode() : loc(0), g_val(0), h_val(0), parent(nullptr), timestep(0), in_openlist(false) {}

SLLNode::SLLNode(int loc, int g_val, int h_val, SLLNode* parent, int timestep, bool in_openlist) :
	loc(loc), g_val(g_val), h_val(h_val), parent(parent), timestep(timestep),
	in_openlist(in_openlist) {}

SLLNode::SLLNode(const SLLNode& other) 
{
	loc = other.loc;
	g_val = other.g_val;
	h_val = other.h_val;
	parent = other.parent;
	timestep = other.timestep;
	in_openlist = other.in_openlist;
}


