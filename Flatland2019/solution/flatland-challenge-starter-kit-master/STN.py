
class test:

    def __init__(self,s):
        self.content1 = s

    def testing(self, content):
        print("hello hello", content, self.content1)
        return

class STN:
    def __init__(self):
        self.constraint_graph: nx.DiGraph = nx.DiGraph()
        self.min_time = -1
        self.end_nodes = []
        self.start_nodes = []
        # self.num_agents = num_agents

    def decode_stn_node(self,in_stn_node: str):
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

    def solve(self):

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