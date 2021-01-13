/*
 * @author: Team An_old_driver
 * @created: 09-2020
 * Copyright (c) 2020 The University of Southern California. All Rights Reserved.
 * Copyrights licensed under an Academic/non-profit use license.
 * See the accompanying LICENSE file for terms.
*/

#include "LLNode.h"

LLNode::LLNode() : loc(0), g_val(0), h_val(0), parent(nullptr), timestep(0) {}

LLNode::LLNode(int loc, int g_val, int h_val, LLNode* parent, int timestep) :
        loc(loc), g_val(g_val), h_val(h_val), parent(parent), timestep(timestep) {}

//LLNode::LLNode(const LLNode& other)
//{
//    loc = other.loc;
//    g_val = other.g_val;
//    h_val = other.h_val;
//    parent = other.parent;
//    timestep = other.timestep;
//    in_openlist = other.in_openlist;
//}


