from flatland.evaluators.client import FlatlandRemoteClient
from flatland.core.env_observation_builder import DummyObservationBuilder
from my_observation_builder import CustomObservationBuilder
from flatland.envs.observations import GlobalObsForRailEnv

from flatland.envs.rail_env_shortest_paths import get_shortest_paths
from flatland.envs.malfunction_generators import malfunction_from_params
from flatland.envs.observations import TreeObsForRailEnv, GlobalObsForRailEnv
from flatland.envs.predictions import ShortestPathPredictorForRailEnv
from flatland.envs.rail_env import RailEnv
from flatland.envs.rail_generators import sparse_rail_generator
from flatland.envs.schedule_generators import sparse_schedule_generator
from flatland.utils.rendertools import RenderTool, AgentRenderVariant

import matplotlib.pyplot as plt
from MapDecoder import convert_flatland_map,linearize_loc
from STN import FlatlandPost

import networkx as nx
from typing import List, Tuple
from logging import warning
from pulp import *
import os
import subprocess
import numpy as np
import time
from flatland.utils.rendertools import RenderTool

class Controller:
    def __init__(self, in_env, idx2node, idx2pose, node2idx):

        print('Controller Initialization')

        self.idx2node = idx2node
        self.idx2pose = idx2pose
        self.node2idx = node2idx

        self.path_list = self.read_file('./config/paths.txt')
        self.n_agent = len(self.path_list)
        self.env = in_env
        self.max_step = 5000

    def add_pause_path_list(self,agent,current_t,max_stop):
        print("change before: ", self.path_list[agent])
        self.path_list[agent] = self.path_list[agent][0:current_t] + [self.path_list[agent][current_t+1]] * max_stop + self.path_list[agent][current_t+1:]
        print("change after: ", self.path_list[agent])

    @staticmethod
    def read_file(file_name=None):
        if type(file_name) == str:
            if file_name.split('.')[-1] == 'pkl':
                with open(file_name, 'rb') as fin:
                    return pickle.load(file=fin)
            elif file_name.split('.')[-1] == 'txt':
                with open(file_name, 'r') as fin:
                    path_str = fin.readlines()
                    temp_path = list()
                    for path in path_str:
                        temp_path.append(list(map(int, path.split(',')[:-1])))
                    return temp_path

        else:
            logging.ERROR('Invalid file name.')
            exit(1)

    def pos2action(self, time_step, agent):
        if time_step >= len(self.path_list[agent]) - 1:  # reach goal
            # print('Agent {0} reach goal'.format(agent))
            return 4

        else:

            # not yet enter start location, wait there.
            if self.path_list[agent][time_step] == -1 and self.path_list[agent][time_step+1] == -1:
                return 4
            # leave station.
            if self.path_list[agent][time_step] == -1 and self.path_list[agent][time_step+1] != -1:
                return 2

            if self.path_list[agent][time_step] == -2:
                return 0



            curr_pos, prev_pos = self.idx2node[self.path_list[agent][time_step]]  # type: tuple
            if self.path_list[agent][time_step+1] == -2:
                c = 1
                while(self.path_list[agent][time_step+1+c] == -2):
                    c+=1
                next_pos = self.idx2pose[self.path_list[agent][time_step + 1 + c]]
            else:
                next_pos = self.idx2pose[self.path_list[agent][time_step + 1]]  # type: tuple
            print(agent, linearize_loc(self.env,curr_pos), linearize_loc(self.env,prev_pos))

            # Relative to global frame, type:np.array
            agent_dir = np.subtract(curr_pos, prev_pos)
            move_dir = np.subtract(next_pos, curr_pos)

            if np.linalg.norm(agent_dir) > 0:
                agent_dir = agent_dir // np.linalg.norm(agent_dir)
            if np.linalg.norm(move_dir) > 0:
                move_dir = move_dir // np.linalg.norm(move_dir)

            # meet deadend
            if (not np.any(agent_dir + move_dir)) and np.linalg.norm(agent_dir) > 0 and np.linalg.norm(move_dir) > 0:
                # print('Agent {0} meets deadend at time step {1}'.format(agent, time_step))
                return 2

            # Transform move direction into agent frame
            else:
                # Relative to agent frame
                out_dir = (agent_dir[0] * move_dir[0] + agent_dir[1] * move_dir[1],
                           -agent_dir[1] * move_dir[0] + agent_dir[0] * move_dir[1])

                # print('out_dir: ', out_dir)
                if out_dir == (0, 0):  # stay
                    return 4
                elif out_dir == (0, 1):  # move left
                    return 1
                elif out_dir == (1, 0):  # move forward
                    return 2
                elif out_dir == (0, -1):  # move right
                    return 3

    def get_actions(self, time_step):
        _actions = {}
        single_agent = -1  # use for single agent movement. e.g. move agent 9 only -> single_agent = 9
        for _idx in range(self.n_agent):
            if single_agent > -1:
                if _idx == single_agent:
                    _actions[_idx] = self.pos2action(time_step, _idx)
                else:
                    _actions[_idx] = 4
            else:
                _actions[_idx] = self.pos2action(time_step, _idx)
        return _actions

class Controller_stn:
    """
    Controller class that reads from STN and produces actions for agents.
    """
    def __init__(self, in_env, idx2node, idx2pose, node2idx, flatland_post):

        print('Controller Initialization')

        self.idx2node = idx2node
        self.idx2pose = idx2pose
        self.node2idx = node2idx
        self.flatland_post = flatland_post
        self.constraint_graph = self.flatland_post.stn.constraint_graph

        self.env = in_env
        self.max_step = 50000

    @staticmethod
    def decode_stn_node(in_stn_node: str):
        """
        Decode the string to integers
        :param in_stn_node: string label of a mapf node
        :return: agent_id, time_step, position
        """
        temp_seg = in_stn_node.split('_')
        return int(temp_seg[0]), int(temp_seg[1]), int(temp_seg[2])

    def pos2action(self,agent, current_t,node_now, node_next):

        print("agent num: ", agent)
        print("end_nodes ", self.flatland_post.stn.end_nodes)
        print("node: ",self.flatland_post.stn.end_nodes[agent])
        if current_t >= int(self.decode_stn_node(self.flatland_post.stn.end_nodes[agent])[1]) - 1:  # reach goal
            # print('Agent {0} reach goal'.format(agent))
            return 4
        else:
            if node_now == -1 and node_next == -1:
                return 4
            if node_now == -1 and node_next != -1:
                return 2

            curr_pos, prev_pos = self.idx2node[node_now]
            next_pos = self.idx2pose[node_next]

            # Relative to global frame, type:np.array
            agent_dir = np.subtract(curr_pos, prev_pos)
            move_dir = np.subtract(next_pos, curr_pos)

            if np.linalg.norm(agent_dir) > 0:
                agent_dir = agent_dir // np.linalg.norm(agent_dir)
            if np.linalg.norm(move_dir) > 0:
                move_dir = move_dir // np.linalg.norm(move_dir)

            # meet deadend
            if (not np.any(agent_dir + move_dir)) and np.linalg.norm(agent_dir) > 0 and np.linalg.norm(move_dir) > 0:
                # print('Agent {0} meets deadend at time step {1}'.format(agent, time_step))
                return 2

            # Transform move direction into agent frame
            else:
                # Relative to agent frame
                out_dir = (agent_dir[0] * move_dir[0] + agent_dir[1] * move_dir[1],
                           -agent_dir[1] * move_dir[0] + agent_dir[0] * move_dir[1])

                # print('out_dir: ', out_dir)
                if out_dir == (0, 0):  # stay
                    return 4
                elif out_dir == (0, 1):  # move left
                    return 1
                elif out_dir == (1, 0):  # move forward
                    return 2
                elif out_dir == (0, -1):  # move right
                    return 3

    # @staticmethod
    #
    # def pos2action(self, time_step, agent):
    #
    #     if time_step >= len(self.path_list[agent]) - 1:  # reach goal
    #         # print('Agent {0} reach goal'.format(agent))
    #         return 4
    #
    #     else:
    #
    #         # not yet enter start location, wait there.
    #         if self.path_list[agent][time_step] == -1 and self.path_list[agent][time_step+1] == -1:
    #             return 4
    #         # leave station.
    #         if self.path_list[agent][time_step] == -1 and self.path_list[agent][time_step+1] != -1:
    #             return 2
    #
    #         if self.path_list[agent][time_step] == -2:
    #             return 0
    #
    #
    #
    #         curr_pos, prev_pos = self.idx2node[self.path_list[agent][time_step]]  # type: tuple
    #         if self.path_list[agent][time_step+1] == -2:
    #             c = 1
    #             while(self.path_list[agent][time_step+1+c] == -2):
    #                 c+=1
    #             next_pos = self.idx2pose[self.path_list[agent][time_step + 1 + c]]
    #         else:
    #             next_pos = self.idx2pose[self.path_list[agent][time_step + 1]]  # type: tuple
    #         print(agent, linearize_loc(self.env,curr_pos), linearize_loc(self.env,prev_pos))
    #
    #         # Relative to global frame, type:np.array
    #         agent_dir = np.subtract(curr_pos, prev_pos)
    #         move_dir = np.subtract(next_pos, curr_pos)

            # if np.linalg.norm(agent_dir) > 0:
            #     agent_dir = agent_dir // np.linalg.norm(agent_dir)
            # if np.linalg.norm(move_dir) > 0:
            #     move_dir = move_dir // np.linalg.norm(move_dir)
            #
            # # meet deadend
            # if (not np.any(agent_dir + move_dir)) and np.linalg.norm(agent_dir) > 0 and np.linalg.norm(move_dir) > 0:
            #     # print('Agent {0} meets deadend at time step {1}'.format(agent, time_step))
            #     return 2
            #
            # # Transform move direction into agent frame
            # else:
            #     # Relative to agent frame
            #     out_dir = (agent_dir[0] * move_dir[0] + agent_dir[1] * move_dir[1],
            #                -agent_dir[1] * move_dir[0] + agent_dir[0] * move_dir[1])
            #
            #     # print('out_dir: ', out_dir)
            #     if out_dir == (0, 0):  # stay
            #         return 4
            #     elif out_dir == (0, 1):  # move left
            #         return 1
            #     elif out_dir == (1, 0):  # move forward
            #         return 2
            #     elif out_dir == (0, -1):  # move right
            #         return 3

    # def get_actions(self, time_step):
    #     _actions = {}
    #     single_agent = -1  # use for single agent movement. e.g. move agent 9 only -> single_agent = 9
    #     for _idx in range(self.n_agent):
    #         if single_agent > -1:
    #             if _idx == single_agent:
    #                 _actions[_idx] = self.pos2action(time_step, _idx)
    #             else:
    #                 _actions[_idx] = 4
    #         else:
    #             _actions[_idx] = self.pos2action(time_step, _idx)
    #     return _actions
