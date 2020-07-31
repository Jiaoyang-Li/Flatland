#!/usr/bin/env python

#Compile codes in PythonCBS in folder CBS-corridor with cmake
from libPythonCBS import PythonCBS
# First of all we import the Flatland rail environment
from flatland.envs.rail_env import RailEnv
from flatland.envs.rail_generators import sparse_rail_generator,rail_from_file
from flatland.envs.schedule_generators import sparse_schedule_generator,schedule_from_file
from flatland.envs.malfunction_generators  import malfunction_from_params, MalfunctionParameters,malfunction_from_file

from flatland.evaluators.client import FlatlandRemoteClient
from flatland.core.env_observation_builder import DummyObservationBuilder
from my_observation_builder import CustomObservationBuilder
from flatland.envs.observations import GlobalObsForRailEnv

from flatland.envs.agent_utils import RailAgentStatus
from flatland.envs.rail_env_shortest_paths import get_shortest_paths
from flatland.envs.predictions import ShortestPathPredictorForRailEnv
from flatland.utils.rendertools import RenderTool, AgentRenderVariant

from Controllers import MCP_Controller
import os
import subprocess
import numpy as np
import time

framework = "LNS"  # "LNS" for large neighborhood search or "GPP" for group prioritized planning
f_w = 1
debug_print = True
remote_test = False
env_renderer_enable = True
input_pause_renderer = False
timelimit = 0  # unit: seconds
default_group_size = 16 # max number of agents in a group. Suggest 8
corridor_method = 1 # or 0/off or 2/reasonable corridor. Suggest 1
chasing = True # helps when speed =1, however takes more time on corridor reasoning.
accept_partial_solution = True
agent_priority_strategy = 0  #  choose a number between 0 and 5. Suggest 1
#                               0: keep the original ordering
#                               1: prefer max speed then max distance
#                               2: prefer min speed then max distance
#                               3: prefer max speed then min distance
#                               4: prefer min speed then min distance
#                               5: prefer different start locations then max speed then max distance
neighbor_generation_strategy = 2    # 0: random walk; 1: start; 2: intersection;
prirority_ordering_strategy = 0     # 0: random; 1: max regret;
replan_strategy = 1                 # 0: CBS; 1: prioritized planning;


#####################################################################
# malfunction parameters
#####################################################################
malfunction_rate = 1/200          # fraction number, probability of having a stop.
min_duration = 3
max_duration = 20


#####################################################################
# Main evaluation loop
#
# This iterates over an arbitrary number of env evaluations
#####################################################################

evaluation_number = 0 # evaluation counter

# path = '/mnt/d/Flatland/test-neurips2020-round1-v1/'
path = '/home/rdaneel/Flatland/test-neurips2020-round1-v1/'
import os
import json
results = []
# Uncomment the following lines to continue previous experiments
# with open("summary.txt", 'r') as file:
#     results = json.load(file)  # load existing results

stochastic_data = MalfunctionParameters(
    malfunction_rate=malfunction_rate,  # Rate of malfunction occurence
    min_duration=min_duration,  # Minimal duration of malfunction
    max_duration=max_duration  # Max duration of malfunction
)


def linearize_loc(in_env, loc):
    """
    This method linearize the locaiton(x,y) into an int.
    :param in_env: local environment of flatland
    :param loc: locaiton pair(x,y)
    :return: linearized locaiton, int.
    """
    return loc[0]*in_env.width + loc[1]


for folder in sorted(os.listdir(path)):
    if os.path.isfile(path + folder):
        continue
    for filename in os.listdir(path+folder):
        file_name,extension = os.path.splitext(filename)
        if extension != '.pkl' and extension != '.mpk':
            continue

        # Avoid solving the same problem again
        skip = False
        algo_name = framework + "_CBSH(1.0)_groupsize=" + str(default_group_size) + "_priority=" + str(agent_priority_strategy)
        for r in results:
            if r['algorithm'] == algo_name and r['instance'] == folder + "_" + file_name:
                skip = True
                print("skip {} with {}".format(algo_name, folder + "_" + file_name))
        if skip:
            continue

        print("\n\n Instance " + folder + '/' + filename)

        #####################################################################
        # step loop information
        #####################################################################
        time_taken_by_controller = []
        time_taken_per_step = []
        steps = 0
        my_observation_builder = GlobalObsForRailEnv()

        # Construct the enviornment from file
        test = path + folder + '/' + filename
        local_env = RailEnv(  
                    width=1,height=1,
                    rail_generator=rail_from_file(test),
                    schedule_generator=schedule_from_file(test),
                    malfunction_generator_and_process_data=malfunction_from_params(stochastic_data),
                    obs_builder_object=GlobalObsForRailEnv(),
                    remove_agents_at_target=True,
                    record_steps=True)
        local_env.reset()

        CBS = PythonCBS(local_env, framework, "CBSH", timelimit, default_group_size, False, f_w,
                corridor_method, chasing, accept_partial_solution, agent_priority_strategy,
                neighbor_generation_strategy, prirority_ordering_strategy, replan_strategy)
        success = CBS.search()
        paths = CBS.getResult()

        # This is for MCP
        if debug_print:
            time_temp = time.time()
        CBS.buildMCP()
        if debug_print:
            print('Time for building MCP: ', time.time() - time_temp)
        if debug_print:
            # print paths
            for i,p in enumerate(paths):
                print(i, ": " , p)

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
            for i, a in enumerate(local_env.agents):
                if(a.status == RailAgentStatus.DONE_REMOVED):
                    # print("---- agent " , i, "done! -----")
                    curr_locs.append(linearize_loc(local_env, a.target))
                elif(a.status == RailAgentStatus.READY_TO_DEPART):
                    curr_locs.append(-1)
                else:
                    curr_locs.append(linearize_loc(local_env, a.position))

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

                for i, a in enumerate(local_env.agents):
                    # print(a.status)
                    # print(a.malfunction_data)
                    if(a.position != None):
                        print(i, a.position, linearize_loc(local_env,a.position))
                    else:
                        print(i, a.position, a.status, "Agent hasn't entered the start location/has reached goal location")

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
                        print(i, a.position, a.status, "Agent hasn't entered the start location/has reached goal location")

            new_curr_locs = []
            for i, a in enumerate(local_env.agents):
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

                if debug_print:
                    CBS.printAllMCP()
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

