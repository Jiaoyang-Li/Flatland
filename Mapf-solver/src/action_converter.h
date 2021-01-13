/*
 * @author: Team An_old_driver
 * @created: 09-2020
 * Copyright (c) 2020 The University of Southern California. All Rights Reserved.
 * Copyrights licensed under an Academic/non-profit use license.
 * See the accompanying LICENSE file for terms.
*/

#include <boost/python.hpp>
#include <numeric>
#include <cmath>
#include <vector>
#include <iostream>
using namespace std;

namespace p = boost::python;

class ActionConverter{
public:
    int num_agent =0;
    int env_width=0;

    ActionConverter(){}
    ActionConverter(int num_agent, int env_width){num_agent = num_agent; env_width = env_width;}

    int pos2action(int curr_loc, int prev_loc, int next_loc){


        if (next_loc == curr_loc){
            return 4;
        }
        if(curr_loc == -1 && next_loc != -1){
            return 2;
        }

        int prev_pos0 = loc0(prev_loc);
        int prev_pos1 = loc1(prev_loc);

//        cout << p::extract<int>(p::long_(prev_loc)) << endl;

        int curr_pos0 = loc0(curr_loc);
        int curr_pos1 = loc1(curr_loc);

        int next_pos0 = loc0(next_loc);
        int next_pos1 = loc1(next_loc);

//        cout << i << endl;
//        cout << prev_pos0 << " " << prev_pos1 << ", " << curr_pos0 << " " << curr_pos1 << ", " << next_pos0 << " " << next_pos1 << endl;

        int agent_dir0 = curr_pos0 - prev_pos0;
        int agent_dir1 = curr_pos1 - prev_pos1;


        complex<int> agent_dir_v (agent_dir0, agent_dir1);

        int move_dir0 = next_pos0 - curr_pos0;
        int move_dir1 = next_pos1 - curr_pos1;

        complex<int> move_dir_v (move_dir0, move_dir1);

        float norm1 = get_norm(agent_dir_v);
        float norm2 = get_norm(move_dir_v);

//        cout << agent_dir0 <<" " << agent_dir1 << ", " << move_dir0 << " " << move_dir1 << ", " << endl;
//
//        cout << norm1 <<", " << norm2 << endl;

        if(norm1>0){
            agent_dir0 = floor(agent_dir0 / norm1);
            agent_dir1 = floor(agent_dir1 / norm1);
        }
        if(norm2>0){
            move_dir0 = floor(move_dir0 / norm2);
            move_dir1 = floor(move_dir1 / norm2);

        }

        // need to recalculate norm; this part can be further optmized.

        complex<int> agent_dir_v2 (agent_dir0, agent_dir1);
        complex<int> move_dir_v2 (move_dir0, move_dir1);

        float norm3 = get_norm(agent_dir_v2);
        float norm4 = get_norm(move_dir_v2);

//        cout << norm3 <<", " << norm4 << endl;

        int a_m_sum0 = agent_dir0 + move_dir0;
        int a_m_sum1 = agent_dir1 + move_dir1;

        // assume it's all 0 and 1.
        if(!(a_m_sum0 | a_m_sum1) && norm3>0 && norm4>0){
            return 2;
        }
        else{
            int out_dir0 = agent_dir0 * move_dir0 + agent_dir1 * move_dir1;
            int out_dir1 = -agent_dir1 * move_dir0 + agent_dir0 * move_dir1;

            if(out_dir0 == 0){
                if(out_dir1 == 0){
                    return 4;
                }
                if(out_dir1 == 1){
                    return 1;
                }
                if(out_dir1 == -1){
                    return 3;
                }
            }
            if(out_dir0 == 1){
                if(out_dir1 == 0)
                    return 2;
            }
        }
        assert(false); // this should never happen
        return -1;
    }


private:
    inline int linearize_loc(int loc0, int loc1) const{
        return loc0 * env_width + loc1;
    }

    inline int loc0(int id) const { return floor(id / (float)env_width); }
    inline int loc1(int id) const { return ((id % env_width) + env_width) % env_width; }


    inline float get_norm(complex<int> n) const {return sqrt(norm(n));}
};