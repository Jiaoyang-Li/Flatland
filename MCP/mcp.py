#! /home/rdaneel/anaconda3/envs/FlatlandChallenge/bin/python
# -*- coding: UTF-8 -*-

import json
import logging
import pickle
from os.path import split
from time import sleep
from typing import AnyStr, Dict, List, Tuple

import networkx as nx
from flatland.envs.rail_env import RailEnvActions
from networkx.algorithms.dag import transitive_reduction
from networkx.exception import NetworkXError
import matplotlib.pyplot as plt
import time


class MCPNode:
    def __init__(self, loc, t):
        self.loc = loc
        self.discrete_time: int = t
        self.counter: int = 0

    def update_counter(self):
        self.counter += 1


class MCP:
    def __init__(self, in_file_name: str):
        # Initialize agent paths and actions
        self.global_timestep = 0
        self.agent_paths = dict()
        self.agent_actions = dict()
        self.agent_timestep = dict()
        self.cur_loc = dict()
        self.start_loc: Dict[int, Tuple[int, int]] = dict()
        self.goal_loc: Dict[int, Tuple[int, int]] = dict()

        with open(in_file_name, 'rb') as fin:
            path_lists = json.load(fin)

        for (_a, p) in enumerate(path_lists):
            self.agent_paths[_a] = p
            self.agent_actions[_a] = RailEnvActions.DO_NOTHING
            self.cur_loc[_a] = -1
            self.agent_timestep[_a] = -1

        for _a in self.agent_paths.keys():
            for (_t, _loc) in enumerate(self.agent_paths[_a]):
                if _loc != -1:
                    self.start_loc[_a] = (_t, _loc)
                    break
            self.goal_loc[_a] = (len(self.agent_paths[_a]) - 1, self.agent_paths[_a][-1])

        self.mcp_graph = nx.DiGraph()
        self.num_of_agents = len(path_lists)
        self.cur_timestep = 0

        # print(self.start_loc)
        # print('---------------------------------------------------')
        # print(self.goal_loc)

    def encode_node(self, agent: int, discrete_time: int):
        return str(agent) + '_' + str(discrete_time)

    def decode_node(self, node_idx):
        return (int(node_idx.split('_')[0]), int(node_idx.split('_')[1]))

    def build(self):
        # Create nodes and type1 edges
        for (_agent, _path) in self.agent_paths.items():
            self.mcp_graph.add_node(self.encode_node(_agent, 0), location=_path[0], counter=0)
            temp_node: str = self.encode_node(_agent, 0)
            for _t in range(1, len(_path)):
                self.mcp_graph.add_node(self.encode_node(_agent, _t), location=_path[_t], counter=0)
                self.mcp_graph.add_edge(temp_node, self.encode_node(_agent, _t))
                # print('Add edge from ', temp_node, ' to ', self.encode_node(_agent, _t))
                temp_node = self.encode_node(_agent, _t)

        # Create type2 edges
        for _a1 in range(0, len(self.agent_paths)):
            for _a2 in range(0, len(self.agent_paths)):
                if _a1 == _a2:
                    continue

                for _t1 in range(len(self.agent_paths[_a1]) - 2):
                    _a1_loc = self.agent_paths[_a1][_t1]
                    if _a1_loc == -1:  # Ignore -1 initial locations
                        continue
                    for _t2 in range(_t1 + 1, len(self.agent_paths[_a2]) - 1):  # x' < x
                        if _a1_loc == self.agent_paths[_a2][_t2 + 1] and self.agent_paths[_a2][_t2 + 1] != -1:
                            self.mcp_graph.add_edge(self.encode_node(_a1, _t1 + 1), self.encode_node(_a2, _t2 + 1))

        # Remove -1 nodes
        _remove = [_node for _node in self.mcp_graph.nodes() if self.mcp_graph.nodes[_node]['location'] == -1]
        self.mcp_graph.remove_nodes_from(_remove)

        # Transitive reduction, assuming self.mcp_graph is a DAG
        # try:
        #     self.mcp_graph = nx.algorithms.dag.transitive_reduction(self.mcp_graph)
        # except NetworkXError as e:
        #     logging.warning(e)
        #     pass
        # nx.draw(self.mcp_graph)
        # plt.show()
        
    def update(self):
        for _a in range(len(self.agent_paths)):
            if self.agent_timestep[_a] == -1:  # agent has not started yet
                _node_idx = self.encode_node(_a, self.start_loc[_a][0])
            else:
                _node_idx = self.encode_node(_a, self.agent_timestep[_a] + 1)
            # print('agent: ', _a)
            # print('_node_idx: ', _node_idx)
            # print('in_degree: ', self.mcp_graph.in_degree[_node_idx])
            # print('counter: ', self.mcp_graph.nodes[_node_idx])

            if self.mcp_graph.in_degree[_node_idx] == self.mcp_graph.nodes[_node_idx]['counter']:
                self.cur_loc[_a] = self.mcp_graph.nodes[_node_idx]['location']
                self.agent_timestep[_a] = self.decode_node(_node_idx)[1]

                # Update counter of its children
                for _child in self.mcp_graph.successors(_node_idx):
                    print(_child)
                    self.mcp_graph.nodes[_child]['counter'] += 1

                # self.mcp_graph.remove_node(_node_idx)

            self.global_timestep += 1
            # print('====================================================================')

        print('Move to: ',  self.cur_loc)



if __name__ == '__main__':
    mcp = MCP('/home/rdaneel/Flatland/test-neurips2020-round1-v1/Test_13/Level_0.json')
    start_time = time.time()
    mcp.build()
    print('Time: ', time.time() - start_time)
    start_time = time.time()
    mcp.update()
    print('Time: ', time.time() - start_time)
    start_time = time.time()

    sleep(1)
    mcp.update()
    sleep(1)
    mcp.update()
    sleep(1)
    mcp.update()
