# coding: utf-8

import networkx as nx
import numpy as np
from typing import List, Tuple


class STN:
    def __init__(self, node_list: list, edge_list: list):
        self._node_list: List = node_list  # add nodes [n0, n1, n2, ...]
        self._edge_list: List[Tuple] = edge_list  # add edge [(n0, n1, {'lb': 1, 'ub': 3})]
        self.constraint_graph = nx.DiGraph()
        self.min_time = -1

    @property
    def node_list(self):
        return self._node_list

    @node_list.setter
    def node_list(self, node_list):
        self._node_list = node_list

    @property
    def edge_list(self):
        return self._edge_list

    @edge_list.setter
    def edge_list(self, edge_list):
        self._edge_list = edge_list

    def build(self):
        self.constraint_graph.add_nodes_from(self._node_list, value=np.inf)
        self.constraint_graph.add_edges_from(self._edge_list)

    def solve(self):
        print(self.constraint_graph.nodes.data())
        print('----')
        # build distance graph
        dist_graph = nx.DiGraph()
        dist_graph.add_nodes_from(self.constraint_graph.nodes, value=np.inf)

        for e in self.constraint_graph.edges.data():
            dist_graph.add_edge(e[0], e[1], weight=e[2]['ub'])
            dist_graph.add_edge(e[1], e[0], weight=-e[2]['lb'])

        shortest_path = nx.floyd_warshall_numpy(dist_graph)
        self.min_time = (-np.transpose(shortest_path)[0].flatten()).tolist()[0]
        print(self.min_time)

        nx.set_node_attributes(self.constraint_graph,
                               values=dict(zip(self.constraint_graph.nodes, self.min_time)), name='value')
        print(self.constraint_graph.nodes.data())

    def remove_node(self, r_node):
        if r_node in self.constraint_graph.nodes:
            self.constraint_graph.remove_node(r_node)


if __name__ == '__main__':
    a = STN(node_list=list(['a', 'b', 'c', 'd', 'e']),
            edge_list=[('a', 'b', {'lb': 10, 'ub': 20}),
                       ('b', 'c', {'lb': 30, 'ub': 40}),
                       ('d', 'c', {'lb': 10, 'ub': 20}),
                       ('d', 'e', {'lb': 40, 'ub': 50}),
                       ('a', 'e', {'lb': 60, 'ub': 70})])
    a.build()
    a.solve()
