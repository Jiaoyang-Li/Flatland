/*
 * @author: Team An_old_driver
 * @created: 09-2020
 * Copyright (c) 2020 The University of Southern California. All Rights Reserved.
 * Copyrights licensed under an Academic/non-profit use license.
 * See the accompanying LICENSE file for terms.
*/

#pragma once
#include <boost/heap/pairing_heap.hpp>
#include <list>
#include <functional>  // for std::hash (c++11 and above)
#include <memory>
#include "common.h"
#include <cmath>
#include <utility>
using namespace std;
using boost::heap::pairing_heap;
using boost::heap::compare;





class LLNode
{
public:

    int loc;
    int g_val;
    float h_val = 0;
    int heading;
    LLNode* parent=nullptr;
    int timestep = 0;
    int time_generated=0;
    int show_time = 0;
    bool in_openlist = false;
    int malfunction_left = 0;

    float position_fraction = 0.0;
    int exit_loc=-1;
    int exit_heading=-1;


    // the following is used to comapre nodes in the OPEN list
    struct compare_node
    {
        // returns true if n1 > n2 (note -- this gives us *min*-heap).
        bool operator()(const LLNode* n1, const LLNode* n2) const
        {
            if (n1->getFVal() == n2->getFVal())
                if (n1->show_time == n2->show_time){
                    return n1->g_val <= n2->g_val;  // break ties towards larger g_vals

                }
                else
                    return n1->show_time < n2->show_time;
            return n1->getFVal() >= n2->getFVal();
        }
    };  // used by OPEN (heap) to compare nodes (top of the heap has min f-val, and then highest g-val)


    // define a typedefs for handles to the heaps (allow up to quickly update a node in the heap)
    typedef boost::heap::pairing_heap< LLNode*, compare<LLNode::compare_node> >::handle_type open_handle_t;
    open_handle_t open_handle;


    LLNode();
    //LLNode(const LLNode& other);
    LLNode(int loc, int g_val, int h_val, LLNode* parent, int timestep);
    inline double getFVal() const { return g_val + h_val; }

    // The following is used by googledensehash for checking whether two nodes are equal
    // we say that two nodes, s1 and s2, are equal if
    // both are non-nullptr and agree on the id and timestep and same heading
    struct eqnode
    {
        bool operator()(const LLNode* s1, const LLNode* s2) const
        {

            return (s1 == s2) || (s1 && s2 &&
                                  s1->loc == s2->loc &&
                                  s1->timestep == s2->timestep &&
                                  s1->heading == s2->heading &&
                                  s1->exit_loc == s2->exit_loc)
                    ;
        }
    };

    // The following is used by googledensehash for generating the hash value of a nodes
    struct NodeHasher
    {
        std::size_t operator()(const LLNode* n) const
        {
            size_t loc_hash = std::hash<int>()(n->loc);
            size_t timestep_hash = std::hash<int>()(n->timestep);
            size_t heading = std::hash<int>()(n->heading);


            return (loc_hash ^ (timestep_hash << 1)*(heading << 1));
        }
    };

};


class SIPPNode: public LLNode
{
public:
    // define a typedefs for handles to the heaps (allow up to quickly update a node in the heap)
    typedef boost::heap::pairing_heap< SIPPNode*, compare<SIPPNode::compare_node> >::handle_type open_handle_t;
    open_handle_t open_handle;
    Interval interval;
    SIPPNode* parent = nullptr;

    SIPPNode() : LLNode() {}
    SIPPNode(int loc, int g_val, int h_val, SIPPNode* parent, int timestep, Interval interval):
            LLNode(loc, g_val, h_val, nullptr, timestep), parent(parent), interval(std::move(interval)) {}

    // The following is used by for generating the hash value of a nodes
    struct NodeHasher
    {
        std::size_t operator()(const SIPPNode* n) const
        {
            size_t loc_hash = std::hash<int>()(n->loc);
            size_t timestep_hash = std::hash<int>()(n->interval.second);
            size_t heading = std::hash<int>()(n->heading);
            return (loc_hash ^ (timestep_hash << 1)*(heading << 1));
        }
    };

    // The following is used for checking whether two nodes are equal
    // we say that two nodes, s1 and s2, are equal if
    // both are non-NULL and agree on the id and timestep
    struct eqnode
    {
        bool operator()(const SIPPNode* n1, const SIPPNode* n2) const
        {
            return (n1 == n2) ||
                   (n1 && n2 && n1->loc == n2->loc &&
                    n1->heading == n2->heading &&
                    n1->interval == n2->interval);
        }
    };
};
