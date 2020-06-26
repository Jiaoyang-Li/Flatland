// Reading an EGraph from a file.
// File is assumed to be in mis format. That is,
// First line p edge int (#vertices) int (#edges)
// Every other line (#edges lines) has: e #vertex1 #vertex2
// (represents a directed edge from vertex1 to vertex2)
// note: vertices starts from 1 (not 0).

#ifndef EGRAPHREADER_H
#define EGRAPHREADER_H

#include <string>
#include <iostream>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <cassert>
#include <fstream>
#include<boost/tokenizer.hpp>
#include <stdlib.h>
#include <stdio.h>
#include <boost/graph/graphviz.hpp>
#include <string>
#include <google/dense_hash_map>
#include <vector>
#include <utility>
#include "map_loader.h"

using namespace std;
//using namespace boost; -- collide with std::tuple
using google::dense_hash_map;      // namespace where class lives by default
using std::tr1::hash;//ext::hash;  // or __gnu_cxx::hash, or maybe tr1::hash, depending on your OS

// this is needed because otherwise we'll have to define the specilized template inside std namespace
struct IntHasher
{
  std::size_t operator()(const int n) const
  {
    using std::tr1::hash;
    //    cout << "COMPUTE HASH: " << *n << " ; Hash=" << hash<int>()(n->id) << endl;
    //cout << "   Pointer Address: " << n << endl;
    return ( hash<int>()(n) );
  }
};

// (BGL bundeled properties)
// this struct contains the internal properties of a vertex
// Since we want a concise representation of EGraphs, we link
// each node in the original search space (Node) with an EGraph (VertexDesc)
struct VertexDesc {
  int node_id;  // in our case, represents the cell id in the grid
  VertexDesc(const int id) {this->node_id = id;}
  VertexDesc() {}  // (needed by boost)
};

// <vertices container , edges container , (un)directed , vertex_propert, edge_property, graph_property>
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS> my_graph;
typedef boost::adjacency_list<boost::vecS, boost::hash_setS, boost::directedS, VertexDesc> my_digraph;
typedef boost::graph_traits<my_digraph>::vertex_descriptor eg_Vertex;
typedef boost::graph_traits<my_digraph>::edge_descriptor eg_Edge;

// typedef boost::graph_traits < boost::adjacency_list <boost::vecS, boost::hash_setS, boost::directedS, VertexDesc> >::vertex_iterator my_vertex_it;
// typedef boost::graph_traits < boost::adjacency_list <boost::vecS, boost::hash_setS, boost::directedS, VertexDesc> >::adjacency_iterator my_edges_it;

// use hash_set to store edges (since the egraph will be queried a lot for edge detection)
/*
struct eqVertex
{
  bool operator()(const eg_Vertex* s1, const eg_Vertex* s2) const
  {
    //    return (s1 == s2) || (s1 && s2 && *s1 == *s2 );
    return (s1 == s2) || (s1 && s2 && s1->node_id == s2->node_id );
  }
};
*/
// we maintain a hash table that maps grid's ids to vertex_descriptor
// then we can do all
class EgraphReader {
 public:
  eg_Vertex u1, u2;  // declaring vertices u1,u2
  eg_Edge e1;  // declaring edge e1
  my_digraph* e_graph;

  // generate hash_map (key is an int (node_id), data is a eg_Vertex (corresponding node on the EGraph),
  //                    check: hasher is hash<int>()
  //                    check: equality of eg_Vertex is corrent
  dense_hash_map< int, eg_Vertex > nodes;  // maps node_id to its vertex_descriptor
  dense_hash_map< int, eg_Vertex >::iterator it;

  EgraphReader();  // generate an empty egraph
  EgraphReader(string fname);
  EgraphReader(int* edgesMap, const MapLoader* ml);
  void printToDOT(string fname);
  bool isEdge(int n1, int n2) const;
  vector< pair<int, int> >* getAllEdges();
  bool containVertex(int n_id) const;
  void removeVertex(int n_id);
  void addEdge(int n1_id, int n2_id);
  void addVertices(const vector<int>* v_list);
  void saveToFile(string fname);
  void createCrissCrossHWY(MapLoader* ml);
  ~EgraphReader();
};

#endif
