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


import networkx as nx
from typing import List, Tuple
from logging import warning
from pulp import *
import os
import subprocess
import numpy as np
import time
from flatland.utils.rendertools import RenderTool




# ----------------------------------------------

# remote_client = FlatlandRemoteClient()



#####################################################################
# Instantiate your custom Observation Builder
# 
# You can build your own Observation Builder by following 
# the example here : 
# https://gitlab.aicrowd.com/flatland/flatland/blob/master/flatland/envs/observations.py#L14
#####################################################################
my_observation_builder = GlobalObsForRailEnv()

# Or if you want to use your own approach to build the observation from the env_step, 
# please feel free to pass a DummyObservationBuilder() object as mentioned below,
# and that will just return a placeholder True for all observation, and you 
# can build your own Observation for all the agents as your please.
# my_observation_builder = DummyObservationBuilder()


#####################################################################
# Main evaluation loop
#
# This iterates over an arbitrary number of env evaluations
#####################################################################
evaluation_number = 0

while True:

    evaluation_number += 1
    # Switch to a new evaluation environemnt
    # 
    # a remote_client.env_create is similar to instantiating a 
    # RailEnv and then doing a env.reset()
    # hence it returns the first observation from the 
    # env.reset()
    # 
    # You can also pass your custom observation_builder object
    # to allow you to have as much control as you wish 
    # over the observation of your choice.
    # time_start = time.time()
    # observation, info = remote_client.env_create(
    #                 obs_builder_object=my_observation_builder
    #             )
    # env_creation_time = time.time() - time_start
    # if not observation:
    #     #
    #     # If the remote_client returns False on a `env_create` call,
    #     # then it basically means that your agent has already been
    #     # evaluated on all the required evaluation environments,
    #     # and hence its safe to break out of the main evaluation loop
    #     break
    #
    # print("Evaluation Number : {}".format(evaluation_number))

    #####################################################################
    # Access to a local copy of the environment
    # 
    #####################################################################
    # Note: You can access a local copy of the environment 
    # by using : 
    #       remote_client.env 
    # 
    # But please ensure to not make any changes (or perform any action) on 
    # the local copy of the env, as then it will diverge from 
    # the state of the remote copy of the env, and the observations and 
    # rewards, etc will behave unexpectedly
    # 
    # You can however probe the local_env instance to get any information
    # you need from the environment. It is a valid RailEnv instance.

    # local_env = remote_client.env

    stochastic_data = {'prop_malfunction':0.01,'malfunction_rate':100,  # Rate of malfunction occurence
                                            'min_duration':3,  # Minimal duration of malfunction
                                            'max_duration':20  # Max duration of malfunction
    }
    # Different agent types (trains) with different speeds.
    speed_ration_map = {1.: 1,  # Fast passenger train
                        1. / 2.: 0,  # Fast freight train
                        1. / 3.: 0,  # Slow commuter train
                        1. / 4.: 0}  # Slow freight train

    local_env = RailEnv(width=30,
                  height=30,
                  rail_generator=sparse_rail_generator(max_num_cities=30,
                                                       # Number of cities in map (where train stations are)
                                                       seed=3,  # Random seed
                                                       grid_mode=False,
                                                       max_rails_between_cities=2,
                                                       max_rails_in_city=8,
                                                       ),
                  schedule_generator=sparse_schedule_generator(speed_ration_map),
                  number_of_agents=30,
                  malfunction_generator_and_process_data=malfunction_from_params(stochastic_data),
                  # Malfunction data generator
                  obs_builder_object=GlobalObsForRailEnv(),
                  remove_agents_at_target=True,
                  record_steps=True
                  )
    observation,info = local_env.reset()
    # print(local_env.agents)
    number_of_agents = len(local_env.agents)


    # ---------------------------------- encode map ---------------------------------------

    original_rail_map = local_env.rail.grid.copy()
    result = MapDecoder(original_rail_map).convert_ori_rail_map()

    max_step = int(4 * 2 * (local_env.width + local_env.height + 20))

    # print(result)

    idx2node = {idx: k for idx, k in enumerate(result.keys())}  # {index:(current, previous)}
    node2idx = {k: idx for idx, k in idx2node.items()}  # {(current, previous) : index}
    idx2pos = {k: v[0] for (k, v) in idx2node.items()}  # {index:current}

    # print(node2idx)

    for n in node2idx:
        print(n, node2idx[n])
    for cur, prev in result:
        for to in result[(cur, prev)]:
            print(cur,prev,to)

    edges = []
    for cur, prev in result:
        for to in result[(cur, prev)]:
            if((to,cur) in node2idx and (cur,prev) in node2idx):
                edges.append((node2idx[(cur, prev)], node2idx[(to, cur)]))

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

    pos2nodes = dict()
    for _, (cur, prev) in idx2node.items():
        if cur not in pos2nodes:
            pos2nodes[cur] = []
        pos2nodes[cur].append((cur, prev))

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


    # -------------- RUN ECBS C++ --------------------------
    # print(max_step)
    # print("ECBS running:")
    # print(os.popen("chmod +x ./ECBS/ECBS").read())
    # res = os.popen("./ECBS/ECBS -m ./config/map.txt -a ./config/agents.txt -o ./config/paths.txt -w 1.1 --makespan "
    #                + str(max_step) + " --debug=1 -g 8")
    # print(res.read())

    # Now we enter into another infinite loop where we
    # compute the actions for all the individual steps in this episode
    # until the episode is `done`
    #
    # An episode is considered done when either all the agents have
    # reached their target destination
    # or when the number of time steps has exceed max_time_steps, which
    # is defined by :

    speed_list = list()
    speed_list_real = list()
    for agent in local_env.agents:
        speed_list.append(1/agent.speed_data['speed'])
        speed_list_real.append(agent.speed_data['speed'])
    print('speed_list: ', speed_list)

    my_post = FlatlandPost(in_env=local_env,in_path = "./config/paths.txt", in_speed_list = speed_list_real)
    my_post.construct_stn()
    # my_post.draw_stn()


    time_taken_by_controller = []
    time_taken_per_step = []
    steps = 0

    my_controller = Controller(local_env, idx2node, idx2pos, node2idx)
    my_controller_stn = Controller_stn(local_env, idx2node, idx2pos, node2idx,my_post)

    print(len(local_env.agents))
    env_renderer = RenderTool(local_env, screen_height=1000,
                              screen_width=1000)

    track = my_post.stn.start_nodes
    while True:
        
        # for agent in local_env.agents:
        #     print(agent.malfunction_data['malfunction'])

        print('step: ', steps)

        #####################################################################
        # Evaluation of a single episode
        #
        #####################################################################
        # Compute the action for this step by using the previously 
        # defined controller
        time_start = time.time()

        mal_agent_idx = []
        mal_agent_steps = []

        for i,a in enumerate(local_env.agents):
            if a.malfunction_data['malfunction'] > 0:

                # print("agent blocked: ", a)
                mal_agent_idx.append(i)
                mal_agent_steps.append(a.malfunction_data['malfunction'])

        # print(mal_agent_idx)
        # print(mal_agent_steps)
        if(len(mal_agent_idx) != 0 ):
            my_post.update_edge_replan(mal_agent_idx,mal_agent_steps,steps)
        #my_post.draw_stn()

        # max_stop = np.max(mal_agent_steps)
        #
        # if(max_stop>0):
        #     for a in range(number_of_agents):
        #         my_controller.add_pause_path_list(a,steps,max_stop)



        # # --- get actions from stn
        print("current track: ", track)
        action_stn = {}



        for a in range(number_of_agents):
            edges = my_post.stn.constraint_graph.out_edges(track[a])
            if(len(edges) == 0):
                action_stn[a] = 4
            else:
                for e in edges:
                    found = False
                    print("edges: ", a , " ", my_post.decode_stn_node(e[1])[0])
                    if(my_post.decode_stn_node(e[1])[0] == a):
                        found = True
                        type1_next_node =e[1]
                        print(my_post.stn.constraint_graph.nodes.data()[type1_next_node]['value'])
                        #  print("asdfas: ", my_post.stn.constraint_graph.nodes.data()[type1_next_node]['value'])
                        if(steps + 1/speed_list_real[a] == my_post.stn.constraint_graph.nodes.data()[type1_next_node]['value']):
                            action_stn[a] = my_controller_stn.pos2action(a,steps,my_controller_stn.decode_stn_node(track[a])[2],my_controller_stn.decode_stn_node(type1_next_node)[2])
                            track[a] = type1_next_node
                        else:
                            action_stn[a] = 0
                        break
                    if (not found):
                        action_stn[a] = 0



        action = my_controller.get_actions(steps)


        steps += 1


        #print("stn action: ", action_stn)
        print("path_action:", action)
        # string = ""
        # for i in  range(len(local_env.agents)):
        #     # print(local_env.agents[i].position)
        #     s = -1
        #     if local_env.agents[i].position == None:
        #         s = -1
        #     else:
        #         s = linearize_loc(local_env,local_env.agents[i].position)
        #     string += str(i) + ": " + str(s) + " "
        #
        # string += "\n"
        # print(string)

        # action = my_controller(observation, number_of_agents)


        time_taken = time.time() - time_start
        time_taken_by_controller.append(time_taken)

        # Perform the chosen action on the environment.
        # The action gets applied to both the local and the remote copy 
        # of the environment instance, and the observation is what is 
        # returned by the local copy of the env, and the rewards, and done and info
        # are returned by the remote copy of the env
        time_start = time.time()

        env_renderer.render_env(show=True, frames=False, show_observations=False)
        local_env.step(action_stn)

        # observation, all_rewards, done, info = remote_client.env_step(action)



        time_taken = time.time() - time_start
        time_taken_per_step.append(time_taken)


        # time.sleep(0.4)
        input()
        # if done['__all__']:
        #     print("Reward : ", sum(list(all_rewards.values())))
        #     #
        #     # When done['__all__'] == True, then the evaluation of this
        #     # particular Env instantiation is complete, and we can break out
        #     # of this loop, and move onto the next Env evaluation
        #     break
    
    np_time_taken_by_controller = np.array(time_taken_by_controller)
    np_time_taken_per_step = np.array(time_taken_per_step)
    print("="*100)
    print("="*100)
    print("Evaluation Number : ", evaluation_number)
    print("Current Env Path : ", remote_client.current_env_path)
    print("Env Creation Time : ", env_creation_time)
    print("Number of Steps : ", steps)
    print("Mean/Std of Time taken by Controller : ", np_time_taken_by_controller.mean(), np_time_taken_by_controller.std())
    print("Mean/Std of Time per Step : ", np_time_taken_per_step.mean(), np_time_taken_per_step.std())
    print("="*100)

print("Evaluation of all environments complete...")
########################################################################
# Submit your Results
# 
# Please do not forget to include this call, as this triggers the 
# final computation of the score statistics, video generation, etc
# and is necesaary to have your submission marked as successfully evaluated
########################################################################
print(remote_client.submit())
