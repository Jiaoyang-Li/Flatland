#!/usr/bin/env python
# coding: utf-8

# This notebook serves for the purpose of testing code
# The content will be transferred to run.py for submission and evaluation

import numpy as np
import time
import logging
import sys

from flatland.utils.rendertools import RenderTool
from flatland.envs.observations import GlobalObsForRailEnv
from flatland.envs.rail_generators import complex_rail_generator
from flatland.envs.schedule_generators import complex_schedule_generator
from flatland.envs.rail_env import RailEnv


class MapDecoder:
    """
    straightward implementation, could be optimized
    The node id is in the form (x,y), could be a linear mapping from
    the most leftupper cornner to the most rightlower corner
    Flatland map encoding scheme, NESW -> N:[NESW], E:[NESW], S[NESW], W[NESW]
    Deadend cell is different, agent faces the opposite to the direction that it can goes.
    """

    def __init__(self, original_map):
        self.original_map = original_map
        self.converted_input = {}

    # Determine next nodes for current_node.
    @staticmethod
    def case_matching(current_node, short_bits):
        next_nodes = []

        if int(short_bits[0]) == 1:  # North
            next_node = (current_node[0]-1, current_node[1])
            next_nodes.append(next_node)
            # print("N", next_node)

        elif int(short_bits[1]) == 1:  # East
            next_node = (current_node[0], current_node[1]+1)
            next_nodes.append(next_node)
            # print("E", next_node)

        elif int(short_bits[2]) == 1:  # South
            next_node = (current_node[0]+1, current_node[1])
            next_nodes.append(next_node)
            # print("S", next_node)

        elif int(short_bits[3]) == 1:  # West
            next_node = (current_node[0], current_node[1]-1)
            next_nodes.append(next_node)
            # print("W", next_node)

        else:
            logging.ERROR('Invalid encoding')
            exit(1)

        return next_nodes

    def slice_16_bits(self, current_node, bits):
        """
        First find the possible previous nodes, and call 'case_matching' function to find the possible next nodes.
        :param current_node:
        :param bits:
        :return:
        """

        previous_nodes = {}

        if int(bits) > 0:  # check previously whether the gird has a train
            # Facing North
            if bits[0:4] != "0000":
                next_nodes = self.case_matching(current_node, bits[0:4])
                # add one previous node, and what nodes the agent can go from current node.
                previous_nodes[(current_node[0]+1, current_node[1])] = next_nodes
                # print("N, and returned next nodes", next_nodes)
                # print("N, and current previous nodes", previous_nodes)

            # East
            elif bits[4:8] != "0000":
                next_nodes = self.case_matching(current_node, bits[4:8])
                previous_nodes[(current_node[0], current_node[1]-1)] = next_nodes
                # print("E",next_nodes)
                # print("E, and current previous nodes", previous_nodes)

            # South
            elif bits[8:12] != "0000":
                next_nodes = self.case_matching(current_node, bits[8:12])
                # print("S",next_nodes)
                previous_nodes[(current_node[0]-1, current_node[1])] = next_nodes
                # print("S, and current previous nodes", previous_nodes)

            # West
            elif bits[12:16] != "0000":
                next_nodes = self.case_matching(current_node, bits[12:16])
                # print("W",next_nodes)
                previous_nodes[(current_node[0], current_node[1]+1)] = next_nodes
                # print("W, and current previous nodes", previous_nodes)

            else:
                logging.ERROR('Invalid decoding')
                exit(1)

        return previous_nodes

    def convert_ori_rail_map(self):
        for row in range(0, self.original_map.shape[0]):
            for col in range(0, self.original_map.shape[1]):
                # Current node: (row,col)
                current_node = (row, col)
                # print("Current node:", (row, col))
                # print('{0:016b}'.format(self.original_map[row][col]))

                # Get previous node, decided by which direction the agent is facing, EXCEPT the deadend nodes.
                previous_next_nodes = self.slice_16_bits(current_node, '{0:016b}'.format(self.original_map[row][col]))
                # print(previous_next_nodes.items())
                
                for previous, direction in previous_next_nodes.items():
                    # print("direction", direction)
                    self.converted_input[(current_node, previous)] = direction

        # print(self.converted_input)
        return self.converted_input


def linearize_loc(in_env, loc):
    return loc[0]*in_env.width + loc[1]


def my_controller(n_agent):
    """
    You are supposed to write this controller
    """
    out_action = {}
    for _idx in range(n_agent):
        out_action[_idx] = np.random.randint(0, 5)
    return out_action


def parse_line_path(l):
    return [int(_node) for _node in l.split(",")[:-1]]


if __name__ == '__main__':
    nr_start_goal = 10  # number of start and goal connections
    nr_extra = 2  # number of extra railway elements added
    min_dist = 8  # minimum grid distance between start and goal
    max_dist = 99999  # maximum grid distance between start and goal
    nr_agent = 10  # number of agents

    # seed = np.random.randint(0, 1000)
    seed = 0

    """
    parameters that control the dimensions of the map.
    Rail generator: generates new rail networks on each reset
    simplest rail generators: 
            envs.rail_generators.rail_from_manual_specifications_generator and 
            envs.rail_generators.random_rail_generator.

    GlobalObsForRailEnv:
    obs -> dict(), with keys == agents
    obs[key] -> tuple with length==3:
        0 transition map array with dimensions (env.height, env.width, 16), assuming 16 bits encoding of transitions.
        1 A 3D array (map_height, map_width, 4) with
            - first channel containing the agents position and direction
            - second channel containing the other agents positions and direction
            - third channel containing agent/other agent malfunctions
            - fourth channel containing agent/other agent fractional speeds

        2 A 2D arrays (map_height, map_width, 2) containing respectively the position of the given agent\
            target and the positions of the other agents targets.
    """
    env = RailEnv(width=30,
                  height=30,
                  rail_generator=complex_rail_generator(nr_start_goal, nr_extra, min_dist, max_dist, seed),
                  schedule_generator=complex_schedule_generator(),
                  obs_builder_object=GlobalObsForRailEnv(),
                  number_of_agents=nr_agent)
    obs = env.reset()
    # print('obs: ', type(obs))
    # print(type(obs[0][2]))
    # print(obs[0][0].shape)

    # tool to render environments
    env_renderer = RenderTool(env, gl="PIL")
    env_renderer.render_env(show=True, frames=False, show_observations=False)

    # The original railway map from the complex_rail_generator
    original_rail_map = env.rail.grid.copy()
    # print('type: original_rail_map', type(original_rail_map))
    # print('type: original_rail_map', original_rail_map.shape)
    np.set_printoptions(threshold=sys.maxsize)
    # print(original_rail_map)

    result = MapDecoder(original_rail_map).convert_ori_rail_map()
    print('result')
    print(len(result.keys()))
    print(result[((0, 18), (1, 18))])

    # pretty print result
    # for i in result.items():
    #     print(i, '\n')

    # Obtain the start and target locations
    # Assuming that each agent is going from one assigned start location to one fixed target location.
    # i.e. no multiple targets available for one agent to go to.

    # can be access by env.agents (lists of agents objects with their own information)
    '''
    information include: position (i.e. start location at time step 0), 
                         direction,
                         target,
                         speed_data,
                         malfunction_data,
                         handle,
                         old_direction,
                         old_position     
    '''

    agent_info = {}

    # for index, agent in enumerate(env.agents):
    #     print(index, agent)

    # idx2node: {index:(current, previous)}
    idx2node = {idx: k for idx, k in enumerate(result.keys())}

    # node2idx: {(current, previous) : index}
    node2idx = {k: idx for idx, k in idx2node.items()}

    # idx2pos: {index:current}
    idx2pos = {k: v[0] for (k, v) in idx2node.items()}

    edges = []
    for cur, prev in result:
        # print(cur, prev, result[(cur, prev)])
        for to in result[(cur, prev)]:
            # print(to)
            edges.append((node2idx[(cur, prev)], node2idx[(to, cur)]))

    # convert agent start information from the format "start_position, direction = a number" to edge
    start_idx = []
    for agent in env.agents:
        # print(agent.position)
        # print(agent.direction)  # direction encoding: {0: North, 1: East, 2: South, 3: West}
        current_pos = agent.position
        prev_pos = None

        if agent.direction == 0:
            prev_pos = (current_pos[0]+1, current_pos[1])
        elif agent.direction == 1:
            prev_pos = (current_pos[0], current_pos[1]-1)
        elif agent.direction == 2:
            prev_pos = (current_pos[0]-1, current_pos[1])
        elif agent.direction == 3:
            prev_pos = (current_pos[0], current_pos[1]+1)
        else:
            logging.ERROR('Invalid prev_pos')
            exit(1)

        start_node = (current_pos, prev_pos)
        index = node2idx[start_node]

        start_idx.append(index)
        # print(start_node, index)

    # print(start_idx)

    pos2nodes = dict()
    for _, (cur, prev) in idx2node.items():
        if cur not in pos2nodes:
            pos2nodes[cur] = []
        pos2nodes[cur].append((cur, prev))

    target_idx = []
    for i, agent in enumerate(env.agents):
        goal_nodes = pos2nodes[agent.target]
        # print(goal_nodes)
        if len(goal_nodes) > 1:
            print("Goal node is not a deadend")

        target_idx.append(node2idx[goal_nodes[0]])

    # map file
    node_file = ""
    edge_file = ""

    node_file += str(len(idx2node)) + "\n"
    for idx, node in idx2node.items():
        node_file += str(linearize_loc(env, idx2pos[idx])) + "\n"

    edge_file += str(len(edges)) + "\n"
    for edge in edges:
        edge_file += str(edge[0]) + "," + str(edge[1]) + "\n"

    map_file = node_file + edge_file

    file = open("map.txt", "w")
    file.write(map_file)
    file.close()

    # agent file
    start_goal_pos_file = ""
    start_goal_pos_file += str(len(env.agents)) + "\n"
    for i, agent in enumerate(env.agents):
        start_goal_pos_file += str(start_idx[i]) + "," + str(target_idx[i]) + ",1\n"

    file = open("agents.txt", "w")
    file.write(start_goal_pos_file)
    file.close()

    for step in range(1):
        # Get agents' handles to give actions
        # handles = env.get_agent_handles()
        # e.g. giving two agents actions, action_dict = {handles[0]:0, handles[1]:0}

        _action = my_controller(n_agent=nr_agent)

        # obs, all_rewards, done: dictionary indexed by agents handles
        # values: correspond to the relevant observations, rewards and terminal status for each agent.

        obs, all_rewards, done, _ = env.step(_action)  # environment take one step with the provided actions.

        # show results
        # print("Observation: \n", obs)
        # print("Rewards: {}, [done={}]".format(all_rewards, done))
        # env_renderer.render_env(show=True, frames=False, show_observations=False)
        time.sleep(600)
