# coding: utf-8

import numpy as np
import pickle
import argparse
from stn import STN
from controller import Controller, create_env, get_prefix
from typing import List, Tuple
from flatland.envs.rail_env import RailEnv
import time
import networkx as nx
import matplotlib.pyplot as plt


class FlatlandPost(Controller):
    def __init__(self, in_env: RailEnv, in_prefix: str, in_path: str, in_speed_list: List):
        Controller.__init__(self, in_env=in_env, in_prefix=in_prefix, in_path=in_path)
        self._speed_list = in_speed_list
        self.stn = STN()

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

    @staticmethod
    def encode_stn_node(in_agent_id: int, in_time_step: int, in_position: int):
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
            v_j_0 = self.encode_stn_node(j, 0, self.path_list[j][0])
            mapf_node.append(v_j_0)
            isin_mapf_node[j][0] = True
            v = v_j_0

            for t in range(1, len(self.path_list[j])):
                if self.path_list[j][t-1] != self.path_list[j][t]:
                    v_j_t = self.encode_stn_node(j, t, self.path_list[j][t])
                    mapf_node.append(v_j_t)
                    mapf_edge.append((v, v_j_t, {'lb': 1.0/self._speed_list[j], 'ub': np.inf}))
                    isin_mapf_node[j][t] = True
                    v = v_j_t

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
                                                      {'lb': 1.0/self.speed_list[j], 'ub': np.inf}))
                                    break

        # build STN through networkx
        self.stn.build(in_nodes=mapf_node, in_edges=mapf_edge)
        print('Done! Require time (s): {0:.5f}'.format(time.time() - start_time))
        return

    def draw_stn(self):
        temp_fig = plt.figure(figsize=(12.8, 4.8))
        h_step = 1
        w_step = 1
        temp_pos = dict()
        temp_label = dict()
        for n in self.stn.constraint_graph.nodes:
            temp_agent, temp_time, temp_idx = self.decode_stn_node(n)
            temp_pos[n] = (temp_time*w_step, (len(self.path_list)-temp_agent)*h_step)
            temp_label[n] = temp_idx
        nx.draw_networkx(self.stn.constraint_graph, pos=temp_pos, with_labels=False, node_size=500)
        nx.draw_networkx_labels(self.stn.constraint_graph, pos=temp_pos, labels=temp_label)
        temp_fig.savefig('./stn.png')
        temp_fig.show()
        return

    def replan(self):
        self.stn.solve()

        return


if __name__ == '__main__':
    # add arg parser
    parser = argparse.ArgumentParser(description='Loading config.pkl, map.txt, and agent.txt')
    parser.add_argument('--config', type=str, default=None)
    parser.add_argument('--path', type=str, default='path_test.txt')
    args = parser.parse_args()

    if args.config is None:
        env, obs = create_env()
        file_prefix = None

    else:
        with open(args.config, 'rb') as fin:
            config_dict = pickle.load(fin)

        file_prefix = get_prefix(args.config)
        env, obs = create_env(map_width=config_dict['map_width'],
                              map_height=config_dict['map_height'],
                              nr_agent=config_dict['nr_agent'],
                              nr_start_goal=config_dict['nr_start_goal'],
                              nr_extra=config_dict['nr_extra'],
                              min_dist=config_dict['min_dist'],
                              max_dist=config_dict['max_dist'],
                              seed=config_dict['seed'])

    speed_list = list()
    for _agent in env.agents:
        speed_list.append(_agent.speed_data['speed'])
    print('speed_list: ', speed_list)

    my_post = FlatlandPost(in_env=env, in_prefix=file_prefix, in_path=args.path, in_speed_list=speed_list)
    my_post.construct_stn()

    # Draw STN
    my_post.draw_stn()
