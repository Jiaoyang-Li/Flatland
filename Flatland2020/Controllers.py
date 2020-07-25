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

from typing import List, Tuple
from logging import warning

import os
import subprocess
import numpy as np
import time
from flatland.utils.rendertools import RenderTool
import math

class MCP_Controller:

    def __init__(self, in_env):
        self.env = in_env
        self.num_agents = len(in_env.agents)


    def linearize_loc(self, loc):
        """
        This method linearize the locaiton(x,y) into an int.
        :param in_env: local environment of flatland
        :param loc: locaiton pair(x,y)
        :return: linearized locaiton, int.
        """
        return loc[0] * self.env.width + loc[1]

    def unlinearize(self, location):
        """
        Method to unlinearize locations
        :param location: int, location to unlinearize
        """
        # print(math.floor(location / self.y_dim), location % self.y_dim)
        return math.floor(location / self.env.width), location % self.env.width

    def get_actions(self, prev_locs, next_locs, curr_locs):

        """
        Function called by run.py.
        :param prev_locs: provided by run.py
        :param next_locs: from CBS next locations
        :param curr_locs: obtained from current envirnoment (given by run.py, so this py dones't have to access CBS objects)
        """

        self.prev_locs = prev_locs
        self.next_locs = next_locs

        actions = {}

        for i in range(0,self.num_agents):
            actions[i] = (self.pos2action(i, curr_locs))

        return actions

    def pos2action(self, i, curr_locs):

        """
        Function to get action for each agent based on the information given. (Prev_loc, Curr_loc, Next_loc)
        :param i: the agent id
        :param curr_locs: reference to the curr location list.
        """


        # if next_location = current_location: including doens't move, wait in the station
        if self.next_locs[i] == curr_locs[i]:
            return 4

        # leave station to the start location
        if curr_locs[i] == -1 and self.next_locs[i] != -1:
            return 2

        prev_pos = self.unlinearize(self.prev_locs[i])
        curr_pos = self.unlinearize(curr_locs[i])
        next_pos = self.unlinearize(self.next_locs[i])

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



class The_Controller:
    """
    Old controller, no longer in use.
    """
    def __init__(self, in_env, idx2node, idx2pose, node2idx, paths, max_step,x_dim, y_dim):

        print('Controller Initialization')

        self.idx2node = idx2node
        self.idx2pose = idx2pose
        self.node2idx = node2idx

        # self.path_list = self.read_file('./config/paths.txt')
        self.path_list = paths
        self.n_agent = len(in_env.agents)
        self.env = in_env
        self.max_step = max_step
        self.x_dim = x_dim
        self.y_dim = y_dim

    def unlinear(self, location):
        # print(math.floor(location / self.y_dim), location % self.y_dim)
        return math.floor(location / self.y_dim), location % self.y_dim

    def pos2action(self, time_step, agent):
        if time_step >= len(self.path_list[agent]) - 1:  # reach goal
            print('Agent {0} reach goal'.format(agent))
            return 4

        else:

            # not yet enter start location, wait there.
            if self.path_list[agent][time_step] == -1 and self.path_list[agent][time_step+1] == -1:
                return 4

            # leave station.
            if self.path_list[agent][time_step] == -1 and self.path_list[agent][time_step+1] != -1:
                return 2

            #print(self.path_list[agent][time_step])
            #print(self.unlinear(self.path_list[agent][time_step]))
            #print(self.idx2node)
            # curr_pos, prev_pos = self.idx2node[self.path_list[agent][time_step]]  # type: tuple
            curr_pos = self.unlinear(self.path_list[agent][time_step])
            prev_pos = self.unlinear(self.path_list[agent][time_step-1])
            # curr_pos, prev_pos = self.unlinear(self.path_list[agent][time_step])

            if self.path_list[agent][time_step+1] == -2:
                c = 1
                while(self.path_list[agent][time_step+1+c] == -2):
                    c+=1
                next_pos = self.unlinear(self.path_list[agent][time_step + 1 + c])
            else:
                next_pos = self.unlinear(self.path_list[agent][time_step + 1])  # type: tuple

            #print(agent, linearize_loc(self.env,curr_pos), linearize_loc(self.env,prev_pos))

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

