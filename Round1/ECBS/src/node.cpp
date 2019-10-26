#include "node.h"

Node::Node() : id(0), g_val(0), h_val(0), parent(NULL), timestep(0), num_internal_conf(0), in_openlist(false) {}

Node::Node(int id, int g_val, int h_val, Node* parent, int timestep, int num_internal_conf, bool in_openlist):
    id(id),  g_val(g_val), h_val(h_val), parent(parent), timestep(timestep),
    num_internal_conf(num_internal_conf), in_openlist(in_openlist) {}

Node::Node(const Node& other) 
{
	id = other.id;
	g_val = other.g_val;
	h_val = other.h_val;
	parent = other.parent;
	timestep = other.timestep;
	in_openlist = other.in_openlist;
	open_handle = other.open_handle;
	focal_handle = other.focal_handle;
	num_internal_conf = other.num_internal_conf;
}


std::ostream& operator<<(std::ostream& os, const Node& n) 
{
  if ( n.parent != NULL )
    os << "LOC=" << n.id << " ; TIMESTEP=" << n.timestep << " ; GVAL=" << n.g_val << " ; HVAL=" << n.h_val
       << " ; #CONF="<< n.num_internal_conf << " ; PARENT=" << (n.parent)->id
       << " ; IN_OPEN?" << std::boolalpha << n.in_openlist;
  else
    os << "LOC=" << n.id << " ; TIMESTEP=" << n.timestep << " ; GVAL=" << n.g_val << " ; HVAL=" << n.h_val
       << " ; #CONF="<< n.num_internal_conf << " ; ROOT (NO PARENT)";
  return os;
}
