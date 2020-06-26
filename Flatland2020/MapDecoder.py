from flatland.envs.observations import GlobalObsForRailEnv

from flatland.envs.rail_env_shortest_paths import get_shortest_paths
from flatland.envs.malfunction_generators import malfunction_from_params
from flatland.envs.predictions import ShortestPathPredictorForRailEnv
from flatland.envs.rail_env import RailEnv
from flatland.envs.rail_generators import sparse_rail_generator
from flatland.envs.schedule_generators import sparse_schedule_generator
from flatland.utils.rendertools import RenderTool, AgentRenderVariant

# MapDecoder.py
#       used for translating the flatland map and agent information to the map & agent information for the family of CBS algorithms

def linearize_loc(in_env, loc):
    """
    This method linearize the locaiton(x,y) into an int.
    :param in_env: local environment of flatland
    :param loc: locaiton pair(x,y)
    :return: linearized locaiton, int.
    """
    return loc[0]*in_env.width + loc[1]

def convert_flatland_map(local_env, original_map):
    """
    Given a flatland environment and the rail map of the environment, it returns information that is required by C++ CBS family solver.
    :param local_env: Flatland environment
    :param original_map: Flatland rail_map of the environment
    :return: idx2node, node2idx, idx2pos, edges, start_idx, pos2nodes, goal_loc_linear, num_goals, goal_idx, speed
    """

    result = MapDecoder(original_map).convert_ori_rail_map()

    print("coverted map: \n", result)

    #####################################################################
    # create locations to nodes
    # each node is indexed.
    # nodes and locations are different in flatland problems:
    #   - locations are (x,y) pairs
    #   - nodes are (current_location, previous_location) pairs
    #   - we use nodes because:
    #       - the train can only go forward, turn left, turn right, stay and nothing(keep doing what it is doing)
    #         So that, where the train is heading to matters. We encapsulate the heading of each train in the nodes.
    #
    # idx2node: {index:(current, previous)}
    #   - given an index, find the corresponding node
    # node2idx: {(current, previous) : index}
    #   - given a node, find the corresponding index
    # idx2pos: {index:current}
    #   - given an index, only returns the current location of the corresponding node
    #####################################################################

    # give each (current_location, previous location) pair an index

    idx2node = {idx: k for idx, k in enumerate(result.keys())}  # {index:(current, previous)}
    node2idx = {k: idx for idx, k in idx2node.items()}  # {(current, previous) : index}
    idx2pos = {k: v[0] for (k, v) in idx2node.items()}  # {index:current}

    for n in node2idx:
        print(n, node2idx[n])
    for cur, prev in result:
        for to in result[(cur, prev)]:
            print(cur,prev,to)


    #####################################################################
    # movements on map, create edges between nodes
    # stored in edges
    #####################################################################

    edges = []
    for cur, prev in result:
        for to in result[(cur, prev)]:
            if((to,cur) in node2idx and (cur,prev) in node2idx):
                edges.append((node2idx[(cur, prev)], node2idx[(to, cur)]))

    #####################################################################
    # process start locations of the flatland environment
    # stored in start_idx
    # start_idx:
    #   - stores the indexes of nodes
    #####################################################################

    # convert agent start information from the format "start_position, direction = a number" to edge
    start_idx = []
    for agent in local_env.agents:
        # print(agent.position)
        # print(agent.direction)  # direction encoding: {0: North, 1: East, 2: South, 3: West}
        current_pos = agent.initial_position
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

    #####################################################################
    # Given a position(location), find out what nodes
    #   - used in:
    #       1. processing of goal locations:
    #           goal location is the location of station. There might be more than one way to go to the station
    #####################################################################

    pos2nodes = dict()
    for _, (cur, prev) in idx2node.items():
        if cur not in pos2nodes:
            pos2nodes[cur] = []
        pos2nodes[cur].append((cur, prev))

    #####################################################################
    # Process the goal locations of the flatland instance and the speed information for each agent
    # Note: the goal location in flatland map is the location of a station, and there can be more than one way to reach the station
    #       therefore, there might be multiple goal nodes for one agent
    # goal_loc_linear:      linearized goal location for each agent
    # num_goals:            the number of goal nodes (that can reach the goal station) for each agent
    # goal_idx:             indexes of goal nodes for each agent
    # speed:                speed information for each agent
    #####################################################################

    goal_loc_linear = []
    num_goals = []
    goal_idx = []
    speed = []
    for i, agent in enumerate(local_env.agents):
        goal_nodes = pos2nodes[agent.target]
        # if len(goal_nodes) > 1:
        #     print("Goal node is not a deadend")
        goal_loc_linear.append(linearize_loc(local_env,agent.target))
        num_goals.append(len(goal_nodes))
        temp = []
        for g in goal_nodes:
            temp.append(node2idx[g])
        goal_idx.append(temp)
        speed.append(agent.speed_data['speed'])

    return idx2node, node2idx, idx2pos, edges, start_idx, pos2nodes, goal_loc_linear, num_goals, goal_idx, speed

class MapDecoder:
    """
    The node id is in the form (x,y), could be a linear mapping from
    the most left-upper corner to the most right-lower corner
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
            next_node = (current_node[0] - 1, current_node[1])
            next_nodes.append(next_node)
            # print("N", next_node)

        if int(short_bits[1]) == 1:  # East
            next_node = (current_node[0], current_node[1] + 1)
            next_nodes.append(next_node)
            # print("E", next_node)

        if int(short_bits[2]) == 1:  # South
            next_node = (current_node[0] + 1, current_node[1])
            next_nodes.append(next_node)
            # print("S", next_node)

        if int(short_bits[3]) == 1:  # West
            next_node = (current_node[0], current_node[1] - 1)
            next_nodes.append(next_node)
            # print("W", next_node)

        return next_nodes

    def slice_16_bits(self, current_node, bits):


        previous_nodes = {}

        # Facing North
        if bits[0:4] != "0000":
            next_nodes = self.case_matching(current_node, bits[0:4])
            # add one previous node, and what nodes the agent can go from current node.
            previous_nodes[(current_node[0] + 1, current_node[1])] = next_nodes
            # print("N, and returned next nodes", next_nodes)
            # print("N, and current previous nodes", previous_nodes)

        # East
        if bits[4:8] != "0000":
            next_nodes = self.case_matching(current_node, bits[4:8])
            previous_nodes[(current_node[0], current_node[1] - 1)] = next_nodes
            # print("E",next_nodes)
            # print("E, and current previous nodes", previous_nodes)

        # South
        if bits[8:12] != "0000":
            next_nodes = self.case_matching(current_node, bits[8:12])
            # print("S",next_nodes)
            previous_nodes[(current_node[0] - 1, current_node[1])] = next_nodes
            # print("S, and current previous nodes", previous_nodes)

        # West
        if bits[12:16] != "0000":
            next_nodes = self.case_matching(current_node, bits[12:16])
            # print("W",next_nodes)
            previous_nodes[(current_node[0], current_node[1] + 1)] = next_nodes
            # print("W, and current previous nodes", previous_nodes)

        return previous_nodes

    def convert_ori_rail_map(self):
        for row in range(0, len(self.original_map)):
            for col in range(0, len(self.original_map[row])):
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