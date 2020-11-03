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


agent_percentages = [1.1 for _ in range(400)]  # agent percentages
replan = [(i % 10 != 0) and (10 <= i < 280) for i in range(400)]  # replan or not
max_iterations = [0 for _ in range(400)]  # mex iterations for LNS
for i in range(10, 220):
    max_iterations[i] = 1000

agent_percentages[351] = 0.70
agent_percentages[350] = 0.75
agent_percentages[346] = 0.75
agent_percentages[301] = 0.75
agent_percentages[291] = 0.80
agent_percentages[322] = 0.80
agent_percentages[321] = 0.80
agent_percentages[320] = 0.80
agent_percentages[311] = 0.80
# agent_percentages[241] = 0.85
agent_percentages[331] = 0.85
agent_percentages[340] = 0.85
agent_percentages[341] = 0.85
agent_percentages[349] = 0.90
# agent_percentages[262] = 0.90
agent_percentages[309] = 0.90
# agent_percentages[231] = 0.90
agent_percentages[281] = 0.95
agent_percentages[333] = 0.95
# agent_percentages[233] = 0.95
agent_percentages[304] = 0.95
# agent_percentages[251] = 0.95
agent_percentages[353] = 0.95
agent_percentages[342] = 0.95
agent_percentages[352] = 0.95
agent_percentages[344] = 0.95
agent_percentages[283] = 0.95
agent_percentages[295] = 0.95
agent_percentages[282] = 0.95
agent_percentages[354] = 0.85
agent_percentages[357] = 0.90

replan[142] = False
replan[219] = False
replan[252] = False
agent_percentages[252] = 0.80
replan[254] = False
agent_percentages[254] = 0.95
replan[258] = False
replan[261] = False
agent_percentages[261] = 0.80
replan[263] = False
replan[271] = False
agent_percentages[271] = 0.80
replan[272] = False
replan[274] = False
replan[279] = False
agent_percentages[279] = 0.95


#####################################################################
# Instantiate a Remote Client
#####################################################################
if remote_test:
    remote_client = FlatlandRemoteClient()


#####################################################################
# Main evaluation loop
#
# This iterates over an arbitrary number of env evaluations
#####################################################################

evaluation_number = 0  # evaluation counter
total_time_limit = 8 * 60 * 60
global_time_start = time.time()


while True:

    if remote_test:
        time_start = time.time()
        observation, info = remote_client.env_create(
                        obs_builder_object=DummyObservationBuilder()
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
                      obs_builder_object=DummyObservationBuilder(),
                      remove_agents_at_target=True,
                      record_steps=True
                      )

        # still necessary to reset local environment?
        observation, info = local_env.reset()

    # number_of_agents = len(local_env.agents)

    # max_time_steps = int(4 * 2 * (local_env.width + local_env.height + 20))

    # if debug_print:
    #     print("number of agents: ", number_of_agents)
    #     for i in range(0,number_of_agents):
    #         print("Agent id: ", i, " agent start location: ", local_env.agents[i].initial_position,
    #               " goal_locaiton: ", local_env.agents[i].target)


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
    if remote_client:
        env_width = local_env.width
        env_height = local_env.height


    framework = "LNS"
    debug = False
    # remaining_time = total_time_limit - (time.time() - global_time_start)
    time_limit = 580 # (predict_time_limit/predict_remaining_time) * remaining_time
    default_group_size = 5  # max number of agents in a group
    CBS = PythonCBS(local_env, framework, time_limit, default_group_size, debug, replan[evaluation_number])
    CBS.search(agent_percentages[evaluation_number], max_iterations[evaluation_number])
    evaluation_number += 1
    CBS.buildMCP()


    while True:
        #####################################################################
        # Evaluation of a single episode
        #
        #####################################################################

        action = CBS.getActions(local_env, steps, 3.0)
        _, all_rewards, done, info = remote_client.env_step(action)

        steps += 1

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



