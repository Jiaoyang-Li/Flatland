from Controllers import MCP_Controller

from flatland.evaluators.client import FlatlandRemoteClient
from flatland.core.env_observation_builder import DummyObservationBuilder
from my_observation_builder import CustomObservationBuilder
from flatland.envs.observations import GlobalObsForRailEnv

from flatland.envs.agent_utils import RailAgentStatus
from flatland.envs.rail_env_shortest_paths import get_shortest_paths
from flatland.envs.malfunction_generators import malfunction_from_params,MalfunctionParameters
from flatland.envs.observations import TreeObsForRailEnv, GlobalObsForRailEnv
from flatland.envs.predictions import ShortestPathPredictorForRailEnv
from flatland.envs.rail_env import RailEnv
from flatland.envs.rail_generators import sparse_rail_generator
from flatland.envs.schedule_generators import sparse_schedule_generator
from flatland.utils.rendertools import RenderTool, AgentRenderVariant

from libPythonCBS import PythonCBS

from typing import List, Tuple
from logging import warning

import os
import subprocess
import numpy as np
import time

#####################################################################
# parameter trigger local testing and remote testing
#####################################################################

remote_test = True
save_txt_file = False
debug_print = False
env_renderer_enable = False
input_pause_renderer = False

#####################################################################
# local testing parameters
#####################################################################
x_dim = 30
y_dim = 30

# parameters to sparse_rail_genertor
max_num_stations = 3
given_seed = 12
given_max_rails_between_cities = 2 # not clear what this does
given_max_rails_in_city = 3 # not clear what this does
given_num_agents = 10

# speed profile, 1 -> speed is 1, 1_2 -> speed 0.5, 1_3 -> speed 1/3, etc. sum has to be 1
given_1_speed_train_percentage = 1
given_1_2_speed_train_percentage = 0
given_1_3_speed_train_percentage = 0
given_1_4_speed_train_percentage = 0

# malfunction parameters
malfunction_rate = 0.4          # fraction number, probability of having a stop.
min_duration = 3
max_duration = 20

# temp solution: C++ solver path file
# path_file ="./config/paths.txt"
# def parse_line_path(l):
#     return [int(node) for node in l.split(",")[:-1]]

def linearize_loc(in_env, loc):
    """
    This method linearize the locaiton(x,y) into an int.
    :param in_env: local environment of flatland
    :param loc: locaiton pair(x,y)
    :return: linearized locaiton, int.
    """
    return loc[0]*in_env.width + loc[1]

#####################################################################
# Instantiate a Remote Client
#####################################################################
if remote_test:
    remote_client = FlatlandRemoteClient()

my_observation_builder = GlobalObsForRailEnv()

#####################################################################
# Main evaluation loop
#
# This iterates over an arbitrary number of env evaluations
#####################################################################

evaluation_number = 0  # evaluation counter
num_of_evaluations = 400  # total number of evaluations
total_time_limit = 8 * 60 * 60
global_time_start = time.time()

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

        stochastic_data = MalfunctionParameters(
                                                malfunction_rate=malfunction_rate,  # Rate of malfunction occurence
                                                min_duration=min_duration,  # Minimal duration of malfunction
                                                max_duration=max_duration  # Max duration of malfunction
                                                )

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
                      malfunction_generator_and_process_data=malfunction_from_params(stochastic_data),
                      obs_builder_object=GlobalObsForRailEnv(),
                      remove_agents_at_target=True,
                      record_steps=True
                      )

        # still necessary to reset local environment?
        observation, info = local_env.reset()

    number_of_agents = len(local_env.agents)

    max_time_steps = int(4 * 2 * (local_env.width + local_env.height + 20))

    if debug_print:
        print("number of agents: ", number_of_agents)
        for i in range(0,number_of_agents):
            print("Agent id: ", i, " agent start location: ", local_env.agents[i].initial_position,
                  " goal_locaiton: ", local_env.agents[i].target)



    #####################################################################
    #  speed information
    #   Not used at this moment.
    #####################################################################

    speed_list = list()
    speed_list_real = list()
    for agent in local_env.agents:
        speed_list.append(1/agent.speed_data['speed'])
        speed_list_real.append(agent.speed_data['speed'])
    # print('speed_list: ', speed_list)


    #####################################################################
    # step loop information
    #####################################################################

    time_taken_by_controller = []
    time_taken_per_step = []
    steps = 0

    #####################################################################
    # CBS and MCP are invoked here.
    #
    #####################################################################

    framework = "LNS"
    f_w = 1
    debug = False
    remaining_time = total_time_limit - (time.time() - global_time_start)
    time_limit = remaining_time / (num_of_evaluations - evaluation_number + 1)
    default_group_size = 16  # max number of agents in a group
    corridor_method = 1  # or "corridor2" or ""
    chasing = True
    accept_partial_solution = True
    agent_priority_strategy = 3
    neighbor_generation_strategy = 3
    prirority_ordering_strategy = 0
    replan_strategy = 1

    CBS = PythonCBS(local_env, framework, "CBSH", time_limit, default_group_size, debug, f_w,
                    corridor_method, chasing, accept_partial_solution, agent_priority_strategy,
                    neighbor_generation_strategy, prirority_ordering_strategy, replan_strategy)
    success = CBS.search()
    paths = CBS.getResult()

    if debug_print:
        time_temp = time.time()
    CBS.buildMCP()
    if debug_print:
        print('TIme for building MCP: ', time.time() - time_temp)

    if debug_print:
        # print paths
        for i,p in enumerate(paths):
            print(i, ": " , p)
        # print mcp
        CBS.printAllMCP()

    #####################################################################
    # stn to action controller
    # this is an old stn to action controller for 2019 challenge
    # may need correction/improvement
    #####################################################################

    my_controller = MCP_Controller(local_env)

    # init prev locations to be -1 for each agent. (None of them has left the station yet)
    prev_locs = [-1 for i in range(0,len(local_env.agents))]

    #####################################################################
    # Show the flatland visualization, for debugging
    #####################################################################

    if env_renderer_enable:
        env_renderer = RenderTool(local_env, screen_height=4000,
                                  screen_width=4000)
        env_renderer.render_env(show=True, show_observations=False, show_predictions=False)

    #####################################################################

    while True:
        #####################################################################
        # Evaluation of a single episode
        #
        #####################################################################

        if debug_print:
            print("current step: ", steps)

        time_start = time.time()

        #####################################################################
        # get curr, next locations from mcp
        #####################################################################
        if debug_print:
            CBS.printAgentTime()
        time_temp = time.time()
        
        if debug_print:
            print('TIme for get next location: ', time.time() - time_temp)

        # get curr locations from the environment (observation)
        curr_locs = []
        for i,a in enumerate(local_env.agents):
            if(a.status == RailAgentStatus.DONE_REMOVED):
                # print("---- agent " , i, "done! -----")
                curr_locs.append(linearize_loc(local_env, a.target))
            elif(a.status == RailAgentStatus.READY_TO_DEPART):
                curr_locs.append(-1)
            else:
                curr_locs.append(linearize_loc(local_env, a.position))

        CBS.replan(local_env, steps, 0)
        next_locs = CBS.getNextLoc(curr_locs, steps + 1)
        action = my_controller.get_actions(prev_locs, next_locs, curr_locs)

        if debug_print:
            print("prev_locs", prev_locs)
            print("curr_locs", curr_locs)
            print("next_locs", next_locs)
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
                # print(a.status)
                # print(a.malfunction_data)
                if(a.position != None):
                    print(i, a.position, linearize_loc(local_env,a.position))
                else:
                    print(i, a.position, a.status, "Agent hasn't entered the start locaiton/has reached goal location")

        if remote_test:
            observation, all_rewards, done, info = remote_client.env_step(action)
        else:
            observation, all_rewards, done, info = local_env.step(action)

        if env_renderer_enable:
            env_renderer.render_env(show=True, show_observations=False, show_predictions=False)

        if debug_print:
            print("after moving agent locations: ")
            for i,a in enumerate(local_env.agents):
                # print(a.status)
                # print(a.malfunction_data)
                if (a.position != None):
                    print(i, a.position, linearize_loc(local_env, a.position))
                else:
                    print(i, a.position, a.status, "Agent hasn't entered the start locaiton/has reached goal location")

        new_curr_locs = []
        for i,a in enumerate(local_env.agents):
            if(a.status == RailAgentStatus.DONE_REMOVED):
                # print("---- agent " , i, "done! -----")
                new_curr_locs.append(linearize_loc(local_env, a.target))
            elif(a.status == RailAgentStatus.READY_TO_DEPART):
                new_curr_locs.append(-1)
            else:
                new_curr_locs.append(linearize_loc(local_env, a.position))

        if debug_print:
            print("new curr locations to MCP: ", new_curr_locs)

        if debug_print:
            time_temp = time.time()

        CBS.updateMCP(new_curr_locs, action)

        if debug_print:
            print('Time for update MCP: ', time.time() - time_temp)

        # display the updated mcp
        # if debug_print:
        #     CBS.printAllMCP()

        # update prev_locs
        for i, a in enumerate(local_env.agents):
            if curr_locs[i] == new_curr_locs[i]:
                continue
            else:
                prev_locs[i] = curr_locs[i]

        # hit enter to go next step
        # print("hit enter to execute the next step")
        if input_pause_renderer:
            input()

        # or wait for a period of time

        # time.sleep(0.5)


        steps += 1
        time_taken = time.time() - time_start
        time_taken_per_step.append(time_taken)

        if done['__all__']:
            print("Reward : ", sum(list(all_rewards.values())))
            print("all arrived.")
            CBS.clearMCP()
            #
            # When done['__all__'] == True, then the evaluation of this 
            # particular Env instantiation is complete, and we can break out 
            # of this loop, and move onto the next Env evaluation
            break

    if remote_test:
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
    else:
        break

print("Evaluation of all environments complete...")
########################################################################
# Submit your Results
# 
# Please do not forget to include this call, as this triggers the 
# final computation of the score statistics, video generation, etc
# and is necesaary to have your submission marked as successfully evaluated
########################################################################
if remote_test:
    print(remote_client.submit())



