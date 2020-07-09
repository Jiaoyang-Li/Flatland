from MapDecoder import convert_flatland_map,linearize_loc
from STN import FlatlandPost
from Controllers import The_Controller

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

from libPythonCBS import PythonCBS

import networkx as nx
from typing import List, Tuple
from logging import warning
from pulp import *
import os
import subprocess
import numpy as np
import time

#####################################################################
# parameter trigger local testing and remote testing
#####################################################################

remote_test = False
save_txt_file = False
debug_print = True

#####################################################################
# local testing parameters
#####################################################################
x_dim = 30
y_dim = 30

stochastic_mode = False
speed_mode = False

# parameters to sparse_rail_genertor
max_num_stations = 3
given_seed = 12
given_max_rails_between_cities = 2 # not clear what this does
given_max_rails_in_city = 3 # not clear what this does
given_num_agents = 6

# speed profile, 1 -> speed is 1, 1_2 -> speed 0.5, 1_3 -> speed 1/3, etc. sum has to be 1
given_1_speed_train_percentage = 1
given_1_2_speed_train_percentage = 0
given_1_3_speed_train_percentage = 0
given_1_4_speed_train_percentage = 0

# malfunction parameters
prop_malfunction = 0.01
malfunction_rate = 30
min_duration = 3
max_duration = 20

# temp solution: C++ solver path file
# path_file ="./config/paths.txt"
# def parse_line_path(l):
#     return [int(node) for node in l.split(",")[:-1]]

#####################################################################
# Instantiate a Remote Client
#####################################################################
if remote_test:
    remote_client = FlatlandRemoteClient()

# random action controller
# def my_controller(obs, number_of_agents):
#     _action = {}
#     for _idx in range(number_of_agents):
#         _action[_idx] = np.random.randint(0, 5)
#     return _action

# default controller given by flatland starter kit
# my_observation_builder = CustomObservationBuilder()

my_observation_builder = GlobalObsForRailEnv()

#####################################################################
# Main evaluation loop
#
# This iterates over an arbitrary number of env evaluations
#####################################################################

evaluation_number = 0 # evaluation counter

while True:

    evaluation_number += 1


    if remote_test:
        time_start = time.time()
        observation, info = remote_client.env_create(
                        obs_builder_object=my_observation_builder
                    )
        env_creation_time = time.time() - time_start
        if not observation:
            #
            # If the remote_client returns False on a `env_create` call,
            # then it basically means that your agent has already been
            # evaluated on all the required evaluation environments,
            # and hence its safe to break out of the main evaluation loop
            break

        print("Evaluation Number : {}".format(evaluation_number))

        local_env = remote_client.env

    else: # testing locally, change


        stochastic_data = {
            'prop_malfunction': prop_malfunction,  # Percentage of defective agents
            'malfunction_rate': malfunction_rate,  # Rate of malfunction occurence
            'min_duration': min_duration,  # Minimal duration of malfunction
            'max_duration': max_duration  # Max duration of malfunction
        }

        # Different agent types (trains) with different speeds.
        # the number [0,1] means the percentage of trains with the corresponding movement speed.
        speed_ration_map = {1.: given_1_speed_train_percentage,  # Fast passenger train
                            1. / 2.: given_1_2_speed_train_percentage,  # Fast freight train
                            1. / 3.: given_1_3_speed_train_percentage,  # Slow commuter train
                            1. / 4.: given_1_4_speed_train_percentage}  # Slow freight train

        local_env = RailEnv(width=x_dim,
                      height=y_dim,
                      rail_generator=sparse_rail_generator(max_num_cities=max_num_stations,
                                                           # Number of cities in map (where train stations are)
                                                           seed=given_seed,  # Random seed
                                                           grid_mode=False,
                                                           max_rails_between_cities=given_max_rails_between_cities,
                                                           max_rails_in_city=given_max_rails_in_city,
                                                           ),
                      schedule_generator=sparse_schedule_generator(speed_ration_map),
                      number_of_agents=given_num_agents,
                      # malfunction_generator_and_process_data=malfunction_from_params(malfunction_rate=malfunction_rate, min_duration=min_duration, max_duration= max_duration),
                      # stochastic_data=stochastic_data,                     # something is wrong with the api... can't use the stochastic data for now.
                      # Malfunction data generator
                      obs_builder_object=GlobalObsForRailEnv(),
                      remove_agents_at_target=True, # WOW this parameter!
                      record_steps=True
                      )

        # still necessary to reset local environment?
        observation, info = local_env.reset()

    number_of_agents = len(local_env.agents)

    max_time_steps = int(4 * 2 * (local_env.width + local_env.height + 20))

    #####################################################################
    # translate the flatland map into (E)CBS map
    #####################################################################

    # get a copy of flatland map
    flatland_rail_map = local_env.rail.grid.copy()

    idx2node, node2idx, idx2pos, edges, start_idx, pos2nodes, goal_loc_linear, num_goals, goal_idx, speed = convert_flatland_map(local_env, flatland_rail_map)

    #####################################################################
    # Ioformation for C++ solvers

    # We decided to use Boost.python api for information exchanging.

    # My suggestion is that we can change this part into sending the map & agent information to the C++ solver instead of saving then as the txt files.
    # So in C++, we can keep the maploader and agentloader classes, but changes how they get information from python.
    # Redoing the flatland map processing in C++ may require extra work.

    # I don't know how you want to exchange information between python and C++...
    # I create lists of nodes, edges and other information that you need for the C++ solver
    # I also leave the old method (saving txt files) here.
    #####################################################################

    # I create lists of information that the C++ solver needs here:


    # -------------------------- map information ----------------------------------
    # all_nodes: indexes of all nodes of the flatland map
    # all_edges: all edges of the flatland map (in terms of indexes, e.g. (idx_node1, idx_node2)...)
    # num_nodes: number of nodes
    # num_edges: number of edges
    #
    # I guess you can pass these ioformation to the ECBS maploader via the boost api...


    # python list that contains all nodes (linearized locations!):
    all_nodes = []
    # python list that contains all nodes (linearized locations!):
    all_edges = []

    for idx, node in idx2node.items():
        all_nodes.append(linearize_loc(local_env, idx2pos[idx]))

    for edge in edges:
        all_edges.append((edge[0],edge[1]))

    num_nodes = len(all_nodes)
    num_edges = len(all_edges)

    # ----- debug print -------
    if debug_print:
        print("num_nodes: ", num_nodes)
        print("all_nodes: \n", all_nodes)
        print("num_edges: ", num_edges)
        print("all_edges: \n", all_edges)


    # ------------------------- agent information ------------------------------------
    # num_agents: int,
    #             - how many agents in this instance
    # agent_start_locations: array of int,
    #             - It has the start location for each agent
    # agent_goal_locaiton: array of int,
    #             - has the goal location for each agent
    # agent_goal_numbers: array of int,
    #             - has the number of goal nodes for each agent.
    # agent_goals: array of int.
    #             - has the goal nodes for each agent.

    num_agents = len(local_env.agents)

    # access each agent's information by providing the agent index, 0,1,2,3,4...
    # e.g. agent 0's start location is agent_start_locations[0]
    #                goal location is agent_goal_locaiton[0]
    #                number of goal nodes is agent_goal_numbers[0]
    #                goal nodes  is agent_goals[0]

    agent_start_locations = start_idx
    agent_goal_locaitons = goal_loc_linear      # just locaiton, not node
    agent_goal_numbers = num_goals              # how many goals each agent has, e.g. [2,1,2,1] agent 0 has 2 goal nodes, agent 1 has 1 goal node.
    agent_goals = goal_idx

    if debug_print:
        print("number of agents: ", num_agents)
        for i in range(0,number_of_agents):
            print("Agent id: ", i, " agent start location: ", agent_start_locations[i], " goal_locaiton: ", agent_goal_locaitons[i], " number of goal nodes: ", agent_goal_numbers[i],
                  " agent goals: ", goal_idx[i])

    # ------------------------------ old method that saves txt files---------------------------
    if save_txt_file:

        node_file = ""
        edge_file = ""
        node_file += str(len(idx2node)) + "\n"
        for idx, node in idx2node.items():
            node_file += str(linearize_loc(local_env, idx2pos[idx])) + "\n"

        edge_file += str(len(edges)) + "\n"
        for edge in edges:
            edge_file += str(edge[0]) + "," + str(edge[1]) + "\n"

        map_file = node_file + edge_file

        file = open("./config/map.txt", "w")
        file.write(map_file)
        file.close()

        # agent file
        start_goal_pos_file = ""
        start_goal_pos_file += str(len(local_env.agents)) + "\n"
        #for i, agent in enumerate(local_env.agents):
        for i in range(len(local_env.agents)):
            print(i, local_env.agents[i].initial_position, local_env.agents[i].target)
            start_goal_pos_file += str(start_idx[i]) + "," + str(goal_loc_linear[i]) + "," + str(num_goals[i])
            for goal in goal_idx[i]:

                start_goal_pos_file += "," + str(goal)
            start_goal_pos_file += ","+ str(int(1/speed[i])) + "\n"


        file = open("./config/agents.txt", "w")
        file.write(start_goal_pos_file)
        file.close()

    #####################################################################
    # use C++ solver here, currently showing the old way (via txt files)
    # To-Do:
    # exchange the information via boost api.
    #####################################################################

    # print(max_time_steps)
    # print("ECBS running:")
    # print(os.popen("chmod +x ./ECBS/ECBS").read())
    # res = os.popen("./ECBS/ECBS -m ./map.txt -a ./agents.txt -o ./paths.txt -w 1.1 --makespan "
    #                + str(max_step) + " --debug=1 -g 8")
    # print(res.read())

    #####################################################################
    # Init STN ...
    #
    #####################################################################

    speed_list = list()
    speed_list_real = list()
    for agent in local_env.agents:
        speed_list.append(1/agent.speed_data['speed'])
        speed_list_real.append(agent.speed_data['speed'])
    print('speed_list: ', speed_list)

    # my_post = FlatlandPost(in_env=local_env,in_path = "./config/paths.txt", in_speed_list = speed_list_real)
    # my_post.construct_stn()

    # if debug_print:
    #     my_post.draw_stn()

    #####################################################################
    # step loop
    #####################################################################

    time_taken_by_controller = []
    time_taken_per_step = []
    steps = 0



    #####################################################################
    # temp method to execute the result of the CBS solver
    # reading from a path.txt file. (Change later for API communication)
    #####################################################################

    if(save_txt_file):
        with open(path_file, "r") as f:
            docs = f.readlines()

        paths = [parse_line_path(l) for l in docs]
    else:
        f_w = 1
        debug = True
        k = 1
        timelimit = 240  # unit: seconds
        default_group_size = 16  # max number of agents in a group
        corridor_method = 1  # or "corridor2" or ""
        chasing = True
        accept_partial_solution = True
        agent_priority_strategy = 0
        CBS = PythonCBS(local_env, "CBSH", k, timelimit, default_group_size, debug, f_w,
                        corridor_method, chasing, accept_partial_solution, agent_priority_strategy)
        success = CBS.search()
        paths = CBS.getResult()



    if debug_print:

        for i,p in enumerate(paths):
            print(i, ": " , p)

    #####################################################################
    # stn to action controller
    # this is an old stn to action controller for 2019 challenge
    # may need correction/improvement
    #####################################################################

    my_controller = The_Controller(local_env, idx2node, idx2pos, node2idx, paths, max_time_steps, x_dim, y_dim)

    #####################################################################
    # Show the flatland visualization
    #####################################################################
    env_renderer = RenderTool(local_env, screen_height=4000,
                              screen_width=4000)
    env_renderer.render_env(show=True, show_observations=False, show_predictions=False)

    # track = my_post.stn.start_nodes



    while True:
        #####################################################################
        # Evaluation of a single episode
        #
        #####################################################################
        # Compute the action for this step by using the previously 
        # defined controller

        if debug_print:
            print("current step: ", steps)

        time_start = time.time()

        # random action controller
        # action = my_controller(observation, number_of_agents)

        action = my_controller.get_actions(steps)
        # for i in range(0,number_of_agents):
        #     action[i]=my_controller.pos2action(steps, i)
        print("actions: ", action)

        time_taken = time.time() - time_start

        time_taken_by_controller.append(time_taken)

        # Perform the chosen action on the environment.
        # The action gets applied to both the local and the remote copy 
        # of the environment instance, and the observation is what is 
        # returned by the local copy of the env, and the rewards, and done and info
        # are returned by the remote copy of the env
        time_start = time.time()

        if debug_print:

            print("before moving agent locations: ")
            for i,a in enumerate(local_env.agents):
                if(a.position != None):
                    print(i, a.position, linearize_loc(local_env,a.position))

        if remote_test:
            observation, all_rewards, done, info = remote_client.env_step(action)
        else:
            observation, all_rewards, done, info = local_env.step(action)

        env_renderer.render_env(show=True, show_observations=False, show_predictions=False)

        if debug_print:
            print("after moving agent locations: ")
            for i,a in enumerate(local_env.agents):
                if (a.position != None):
                    print(i, a.position, linearize_loc(local_env, a.position))
        # hit enter to go next step
        # print("hit enter to execute the next step")
        # input()

        # or wait for a period of time

        # time.sleep(0.5)
        input()

        steps += 1
        time_taken = time.time() - time_start
        time_taken_per_step.append(time_taken)

        if done['__all__']:
            print("Reward : ", sum(list(all_rewards.values())))
            print("all arrived.")
            #
            # When done['__all__'] == True, then the evaluation of this 
            # particular Env instantiation is complete, and we can break out 
            # of this loop, and move onto the next Env evaluation
            break
    break
    # np_time_taken_by_controller = np.array(time_taken_by_controller)
    # np_time_taken_per_step = np.array(time_taken_per_step)
    # print("="*100)
    # print("="*100)
    # print("Evaluation Number : ", evaluation_number)
    # print("Current Env Path : ", remote_client.current_env_path)
    # print("Env Creation Time : ", env_creation_time)
    # print("Number of Steps : ", steps)
    # print("Mean/Std of Time taken by Controller : ", np_time_taken_by_controller.mean(), np_time_taken_by_controller.std())
    # print("Mean/Std of Time per Step : ", np_time_taken_per_step.mean(), np_time_taken_per_step.std())
    # print("="*100)

print("Evaluation of all environments complete...")
########################################################################
# Submit your Results
# 
# Please do not forget to include this call, as this triggers the 
# final computation of the score statistics, video generation, etc
# and is necesaary to have your submission marked as successfully evaluated
########################################################################
# print(remote_client.submit())



