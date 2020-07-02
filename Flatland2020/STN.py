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
import networkx as nx
import matplotlib.pyplot as plt
from pulp import *
import os
from logging import warning
import time
import numpy as np
import pickle


class FlatlandPost:
    def __init__(self, in_env: RailEnv, in_path: str, in_speed_list: List):
        self.env = in_env
        self.path_list = self.read_file(in_path)
        self._speed_list = in_speed_list
        self.stn = STN()
        self.n_agent = len(self.path_list)
        self.max_step = int(4 * 2 * (self.env.width + self.env.height + 20))
        self.start_nodes = list()
        self.goal_nodes = list()

    @staticmethod
    def read_file(file_name=None):
        if type(file_name) == str:
            if file_name.split('.')[-1] == 'pkl':
                with open(file_name, 'rb') as _fin:
                    return pickle.load(file=_fin)
            elif file_name.split('.')[-1] == 'txt':
                with open(file_name, 'r') as _fin:
                    path_str = _fin.readlines()
                    temp_path = list()
                    for path in path_str:
                        temp_path.append(list(map(int, path.split(',')[:-1])))
                    return temp_path

        else:
            logging.ERROR('Invalid file name.')
            exit(1)

    @property
    def speed_list(self):
        return self._speed_list

    @speed_list.setter
    def speed_list(self, in_speed: List[float]):
        """
        TODO: need Yi to get the real speed from server
        :param in_speed: may come from the server and transform in this method
        :return:
        """
        self._speed_list = in_speed
        return

    # used when agent stop accidentally for a period of time
    # stop at location (node), add how_long to the lb of the outgoing edges of this node
    def update_edge_replan(self, agent_idx_list, delay_steps, current_time):
        for i, idx in enumerate(agent_idx_list):
            node_name = self.encode_stn_node(idx, current_time, self.path_list[idx][current_time])
            # print(node_name)

            for _edge in self.stn.constraint_graph.out_edges(node_name, data=True):
                _edge[2]['lb'] += delay_steps[i]
        self.stn.solve()

    @staticmethod
    def encode_stn_node(in_agent_id: int, in_time_step: int, in_position: int):
        # print( str(in_agent_id) + '_' + str(in_time_step) + '_' + str(in_position))
        return str(in_agent_id) + '_' + str(in_time_step) + '_' + str(in_position)

    @staticmethod
    def decode_stn_node(in_stn_node: str):
        """
        Decode the string to integers
        :param in_stn_node: string label of a mapf node
        :return: agent_id, time_step, position
        """
        temp_seg = in_stn_node.split('_')
        return int(temp_seg[0]), int(temp_seg[1]), int(temp_seg[2])

    def construct_stn(self):
        start_time = time.time()
        print('Constructing STN for MAPF ...')

        mapf_node: List[str] = list()
        mapf_edge: List[Tuple] = list()
        isin_mapf_node: np.ndarray = np.zeros((self.n_agent, self.max_step), dtype=bool)

        # Create nodes and type1 edges
        for j in range(self.n_agent):
            v_j_0: str = self.encode_stn_node(j, 0, self.path_list[j][0])
            mapf_node.append(v_j_0)
            self.start_nodes.append(v_j_0)
            isin_mapf_node[j][0] = True
            v = v_j_0

            for t in range(1, len(self.path_list[j])):
                if self.path_list[j][t-1] != self.path_list[j][t]:
                    v_j_t = self.encode_stn_node(j, t, self.path_list[j][t])
                    mapf_node.append(v_j_t)
                    mapf_edge.append((v, v_j_t, {'lb': 1.0/self._speed_list[j], 'ub': np.inf}))
                    isin_mapf_node[j][t] = True
                    v = v_j_t

            self.goal_nodes.append(v)

        # Create type2 edges
        for j in range(self.n_agent):
            for t_j in range(0, len(self.path_list[j])):
                if isin_mapf_node[j][t_j]:
                    for k in range(self.n_agent):
                        if j != k:
                            for t_k in range(t_j+1, len(self.path_list[k])):
                                if isin_mapf_node[k][t_k] and self.path_list[j][t_j] == self.path_list[k][t_k]:
                                    mapf_edge.append((self.encode_stn_node(j, t_j, self.path_list[j][t_j]),
                                                      self.encode_stn_node(k, t_k, self.path_list[k][t_k]),
                                                      {'lb': 1.0/self.speed_list[j] * 2, 'ub': np.inf}))
                                    break

        # build STN through networkx
        self.stn.build(in_nodes=mapf_node, in_edges=mapf_edge)
        print('Done! Require time (s): {0:.5f}'.format(time.time() - start_time))

        self.stn.solve()
        return

    def draw_stn(self):


        self.stn.solve()

        temp_fig = plt.figure(figsize=(200, 100))
        h_step = 1
        w_step = 1
        temp_pos = dict()
        temp_label = dict()
        for n in self.stn.constraint_graph.nodes:
            temp_agent, temp_time, temp_idx = self.decode_stn_node(n)
            temp_pos[n] = (temp_time*w_step, (len(self.path_list)-temp_agent)*h_step)
            temp_label[n] = self.stn.constraint_graph.nodes.data()[n]['value']
        nx.draw_networkx(self.stn.constraint_graph, pos=temp_pos, with_labels=False, node_size=800)
        nx.draw_networkx_labels(self.stn.constraint_graph, pos=temp_pos, labels=temp_label)

        edge_label = {}
        for e in self.stn.constraint_graph.edges.data():
            # print("edgegegege", e[2]['lb'])
            edge_label[(e[0],e[1])] = e[2]['lb']

        nx.draw_networkx_edge_labels(self.stn.constraint_graph, pos=temp_pos, labels = edge_label)
        temp_fig.savefig('./stn.png')
        # temp_fig.show()
        return


class STN:
    def __init__(self):
        self.constraint_graph: nx.DiGraph = nx.DiGraph()
        self.min_time = -1
        self.end_nodes = []
        self.start_nodes = []
        # self.num_agents = num_agents

    @staticmethod
    def decode_stn_node(in_stn_node: str):
        """
        Decode the string to integers
        :param in_stn_node: string label of a mapf node
        :return: agent_id, time_step, position
        """
        temp_seg = in_stn_node.split('_')
        return int(temp_seg[0]), int(temp_seg[1]), int(temp_seg[2])

    def build(self, in_nodes: List[str], in_edges: List[Tuple]):
        print('Build STN ...')
        self.constraint_graph.add_nodes_from(in_nodes, value=np.inf)
        self.constraint_graph.add_edges_from(in_edges)
        return

    def solve(self): # I changed this part and solve the STN via linear programming... The old code generates lower bound and upper bound for some nodes, not all nodes.

        end_nodes = []
        # find end nodes
        for n in self.constraint_graph.nodes():

            edges = self.constraint_graph.out_edges(n)

            found = False
            for e in edges:

                if self.decode_stn_node(e[1])[0] == self.decode_stn_node(n)[0]:
                    found = True

            if(not found):
                end_nodes.append(n)
        #print("end_nodes: ", end_nodes)
        self.end_nodes = end_nodes

        start_nodes = []
        # find start nodes
        for n in self.constraint_graph.nodes():
            if self.constraint_graph.in_degree(n) == 0:
                start_nodes.append(n)
        # print(start_nodes)
        self.start_nodes = start_nodes

        # create t variable for each node:
        Xs = {}
        for n in self.constraint_graph.nodes():
            Xs[n] = LpVariable(n,0,100000)

        prob = LpProblem("prob", LpMinimize)

        # create objective function
        obj = 0
        for end in end_nodes:
            obj += Xs[end]
        # print(obj)
        prob += obj

        # constrain 1
        con1 = 0
        for start in start_nodes:
            con1 += Xs[start]
        # print(con1==0)
        prob += con1==0

        # create  constraints
        for e in self.constraint_graph.edges.data():
            con = 0
            # print(e)
            con = Xs[e[1]] - Xs[e[0]] >= e[2]['lb']

            # print(con)
            prob += con

        prob.solve()
        Xs_values = []
        for n in self.constraint_graph.nodes():
            # print(Xs[n].value())
            Xs_values.append(Xs[n].value())

        nx.set_node_attributes(self.constraint_graph,values=dict(zip(self.constraint_graph.nodes,Xs_values)),name='value')

        # the below code seems to generate the lower and upper bounds for the nodes on a shortest path...

        # print(self.constraint_graph.nodes.data())
        # print('----')
        # # build distance graph
        # dist_graph = nx.DiGraph()
        # dist_graph.add_nodes_from(self.constraint_graph.nodes, value=np.inf)
        #
        # for e in self.constraint_graph.edges.data():
        #     dist_graph.add_edge(e[0], e[1], weight=e[2]['ub'])
        #     dist_graph.add_edge(e[1], e[0], weight=-e[2]['lb'])
        #
        # shortest_path = nx.floyd_warshall_numpy(dist_graph)
        # self.min_time = (-np.transpose(shortest_path)[0].flatten()).tolist()[0]
        # print("min_time", self.min_time)
        # #
        # # nx.set_node_attributes(self.constraint_graph,
        # #                        values=dict(zip(self.constraint_graph.nodes, self.min_time)), name='value')
        #
        # for d in self.constraint_graph.nodes.data():
        #     print (d)
        return

    def graph_opt(self):
        print(self.constraint_graph.nodes.data())
        print('----')

        # build distance graph
        dist_graph = nx.DiGraph()
        dist_graph.add_nodes_from(self.constraint_graph.nodes, value=np.inf)

        for _edge in self.constraint_graph.edges.data():
            dist_graph.add_edge(_edge[0], _edge[1], weight=_edge[2]['ub'])
            dist_graph.add_edge(_edge[1], _edge[0], weight=-_edge[2]['lb'])

        # add source node
        dist_graph.add_node('x_s', value=0)
        for _node in self.start_nodes:
            dist_graph.add_edge('x_s', _node, weight=0)
            dist_graph.add_edge(_node, 'x_s', weight=0)

        plan_exec_schedule = dict()
        for _node in dist_graph.nodes:
            plan_exec_schedule[_node] = -nx.bellman_ford_path_length(dist_graph, _node, 'x_s')

        print(plan_exec_schedule)
        for _node in dist_graph:
            print('{0}: {1}'.format(_node, plan_exec_schedule[_node]))

        return


if __name__ == '__main__':
    a = STN()
    a.start_nodes.append('a_1_0')
    a.start_nodes.append('b_2_0')
    a.build(in_nodes=['a_1_0', 'b_1_1', 'c_1_2', 'd_1_3', 'e_1_4', 'b_2_0', 'c_2_1', 'f_2_2', 'c_2_3', 'd_2_4'],
            in_edges=[('a_1_0', 'b_1_1', {'lb': 4, 'ub': np.inf}),
                      ('b_1_1', 'c_1_2', {'lb': 4, 'ub': np.inf}),
                      ('c_1_2', 'd_1_3', {'lb': 4, 'ub': np.inf}),
                      ('d_1_3', 'e_1_4', {'lb': 4, 'ub': np.inf}),
                      ('b_2_0', 'c_2_1', {'lb': 16, 'ub': np.inf}),
                      ('c_2_1', 'f_2_2', {'lb': 16, 'ub': np.inf}),
                      ('f_2_2', 'c_2_3', {'lb': 16, 'ub': np.inf}),
                      ('c_2_3', 'd_2_4', {'lb': 16, 'ub': np.inf}),

                      ('b_2_0', 'b_1_1', {'lb': 0, 'ub': np.inf}),
                      ('c_2_1', 'c_1_2', {'lb': 0, 'ub': np.inf}),
                      ('c_1_2', 'c_2_3', {'lb': 0, 'ub': np.inf}),
                      ('d_1_3', 'd_2_4', {'lb': 0, 'ub': np.inf})])

    a.graph_opt()
