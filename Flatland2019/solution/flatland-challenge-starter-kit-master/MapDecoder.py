# MapDecoder.py
#       used for translating the flatland map and agent information to the map & agent information for the family of CBS algorithms

def linearize_loc(in_env, loc):
    return loc[0]*in_env.width + loc[1]

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