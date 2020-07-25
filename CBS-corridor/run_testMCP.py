#!/usr/bin/env python

#Compile codes in PythonCBS in folder CBS-corridor with cmake

import inspect
import time
from random import random
import numpy as np

# First of all we import the Flatland rail environment
from flatland.envs.rail_env import RailEnv
from flatland.envs.rail_generators import sparse_rail_generator
from flatland.envs.schedule_generators import sparse_schedule_generator
from flatland.envs.malfunction_generators import MalfunctionParameters, malfunction_from_params
from flatland.envs.observations import GlobalObsForRailEnv

# We also include a renderer because we want to visualize what is going on in the environment
from flatland.utils.rendertools import AgentRenderVariant, RenderTool
from libPythonCBS import PythonCBS


def idx2pos(loc, width):
    return np.floor(loc / width), loc % width

def pos2idx(row, col, width):
    return row*width + col

width = 50  # With of map
height = 50  # Height of map
nr_trains = 4  # Number of trains that have an assigned task in the env
cities_in_map = 2  # Number of cities where agents can start or end
seed = 12  # Random seed
grid_distribution_of_cities = False  # Type of city distribution, if False cities are randomly placed
max_rails_between_cities = 2  # Max number of tracks allowed between cities. This is number of entry point to a city
max_rail_in_cities = 3  # Max number of parallel tracks within a city, representing a realistic trainstation

rail_generator = sparse_rail_generator(max_num_cities=cities_in_map,
                                       seed=seed,
                                       grid_mode=grid_distribution_of_cities,
                                       max_rails_between_cities=max_rails_between_cities,
                                       max_rails_in_city=max_rail_in_cities,
                                       )

# The schedule generator can make very basic schedules with a start point, end point and a speed profile for each agent.
# The speed profiles can be adjusted directly as well as shown later on. We start by introducing a statistical
# distribution of speed profiles

# Different agent types (trains) with different speeds.
speed_ration_map = {1.: 1,  # Fast passenger train
                    1. / 2.: 0,  # Fast freight train
                    1. / 3.: 0,  # Slow commuter train
                    1. / 4.: 0}  # Slow freight train

# We can now initiate the schedule generator with the given speed profiles

schedule_generator = sparse_schedule_generator(speed_ration_map)

# We can furthermore pass stochastic data to the RailEnv constructor which will allow for stochastic malfunctions
# during an episode.
stochastic_data = MalfunctionParameters(malfunction_rate=0,  # Rate of malfunction occurence
                                        min_duration=3,  # Minimal duration of malfunction
                                        max_duration=20  # Max duration of malfunction
                                        )
print(stochastic_data)

# Custom observation builder without predictor
observation_builder = GlobalObsForRailEnv()

# Custom observation builder with predictor, uncomment line below if you want to try this one
# observation_builder = TreeObsForRailEnv(max_depth=2, predictor=ShortestPathPredictorForRailEnv())

# Construct the environment with the given observation, generataors, predictors, and stochastic data
env = RailEnv(width=width,
              height=height,
              rail_generator=rail_generator,
              schedule_generator=schedule_generator,
              number_of_agents=nr_trains,
              malfunction_generator_and_process_data=malfunction_from_params(stochastic_data),
              obs_builder_object=observation_builder,
              remove_agents_at_target=True  # Removes agents at the end of their journey to make space for others
              )

# Initiate the renderer
env.reset()

# env_renderer = RenderTool(env, gl="PILSVG",
#                           agent_render_variant=AgentRenderVariant.AGENT_SHOWS_OPTIONS_AND_BOX,
#                           show_debug=False,
#                           screen_height=800,  # Adjust these parameters to fit your resolution
#                           screen_width=800)  # Adjust these parameters to fit your resolution

framework = "LNS"  # "LNS" for large neighborhood search or "GPP" for group prioritized planning
f_w = 1
debug = True
k = 1
timelimit = 240  # unit: seconds
default_group_size = 16 # max number of agents in a group
corridor_method = 1 # or 0/off or 2/reasonable corridor. Suggest 1
chasing = True # helps when speed =1, however takes more time on corridor reasoning.
accept_partial_solution = True
agent_priority_strategy = 0  #  choose a number between 0 and 5
#                               0: keep the original ordering
#                               1: prefer max speed then max distance
#                               2: prefer min speed then max distance
#                               3: prefer max speed then min distance
#                               4: prefer min speed then min distance
#                               5: prefer different start locations then max speed then max distance
CBS = PythonCBS(env, "CBSH", k, timelimit, default_group_size, debug, f_w,
                corridor_method, chasing, accept_partial_solution, agent_priority_strategy)
success = CBS.search()
plan = CBS.getResult()

for p in plan:
    print(p)

# write results to files for performance analysis
fileName = str(env.width) + "x" + str(env.height) + "map_" \
           + str(env.get_num_agents()) + "trains_" \
           + "_groupsize=" + str(default_group_size) \
           + "_seed=" + str(seed) + ".csv"
CBS.writeResultsToFile(fileName)


# Initialize render environment
env_renderer = RenderTool(env,
                          agent_render_variant=AgentRenderVariant.ONE_STEP_BEHIND,
                          show_debug=False,
                          screen_height=600,  # Adjust these parameters to fit your resolution
                          screen_width=800)  # Adjust these parameters to fit your resolution

env_renderer.reset()

env_renderer.render_env(show=True, frames=True, show_observations=False)

CBS.buildMCP()
print("Building MCP in python")
CBS.printMCP()

action_dict = {}
cur_loc = [-1 for _ in range(len(plan))]
pre_loc = [-1 for _ in range(len(plan))]
for step in range(500):
    next_loc = CBS.getNextLoc()

    for agent in range(len(plan)):
        if cur_loc[agent] == next_loc[agent] == plan[agent][-1]:  # Already reach goal
            action_dict[agent] = 4

        elif next_loc[agent] == -1:  # Not enter location
            action_dict[agent] = 4

        elif cur_loc[agent] == -1 and next_loc[agent] != -1:  # appear at the station
            action_dict[agent] = 2

        else:  # Moving
            if pre_loc[agent] == -1:
                pre_pos = idx2pos(cur_loc[agent])
            else:
                pre_pos = idx2pos(pre_loc[agent])
            cur_pos = idx2pos(cur_loc[agent]) 
            next_pos = idx2pos(next_loc[agent])

            # Relative to global frame, type:np.array
            agent_dir = np.subtract(curr_pos, prev_pos)
            move_dir = np.subtract(next_pos, curr_pos)

            if np.linalg.norm(agent_dir) > 0:
                agent_dir = agent_dir // np.linalg.norm(agent_dir)
            if np.linalg.norm(move_dir) > 0:
                move_dir = move_dir // np.linalg.norm(move_dir)

            # meet deadend
            if (not np.any(agent_dir + move_dir)) and np.linalg.norm(agent_dir) > 0 and np.linalg.norm(move_dir) > 0:
                action_dict[agent] = 2

            # Transform move direction into agent frame
            else:
                # Relative to agent frame
                out_dir = (agent_dir[0] * move_dir[0] + agent_dir[1] * move_dir[1],
                            -agent_dir[1] * move_dir[0] + agent_dir[0] * move_dir[1])

                # print('out_dir: ', out_dir)
                if out_dir == (0, 0):  # stay
                    action_dict[agent] = 4
                elif out_dir == (0, 1):  # move left
                    action_dict[agent] = 1
                elif out_dir == (1, 0):  # move forward
                    action_dict[agent] = 2
                elif out_dir == (0, -1):  # move right
                    action_dict[agent] = 3
    print(action_dict)

    obs, all_rewards, done, info = env.step(action_dict)
    # channel = 0
    # for agent in range(len(plan)):
    #     if action_dict[agent] == 4 and cur_loc[agent] == -1:
    #         continue
    #     print(np.where(obs[agent][1][:,:,channel] > -1))
    #     row = np.where(obs[agent][1][:,:,channel] > -1)[0][0]
    #     col = np.where(obs[agent][1][:,:,channel] > -1)[1][0]
    #     print('row: {0}, col: {1}'.format(row, col))
    #     cur_loc[agent] = pos2idx(row, col, width)
    #     print('cur_loc[{0}] = {1}'.format(agent, cur_loc[agent]))

    print("Rewards: ", all_rewards, "  [done=", done, "]")
    env_renderer.render_env(show=True, frames=False, show_observations=False )
    print('*****************************')
    tmp_ag = 3
    channel = 2
    print(obs[tmp_ag][1].shape)
    print(obs[tmp_ag][1][:,:,channel].shape)
    print(np.where(obs[tmp_ag][1][:,:,channel] > -1))
    np.set_printoptions(threshold=np.inf)
    print(obs[tmp_ag][1][:,:,channel])
    print('*****************************')
    time.sleep(1)

    if done["__all__"]:
        print("Done!!")
        break

    else:
        CBS.updateMCP(cur_loc)

    time.sleep(0.2)

x=input("Press any key to exit")


# prob = 0.6
# cur_loc = [-1 for _ in range(len(plan))]
# for t in range(10):
#     print('--------------------------------------------------------')
#     print('At time ', t)
#     print('Current location: ', cur_loc)
#     next_loc = CBS.getNextLoc()
#     print("Next location :", next_loc)
#     mal_function = [random() < prob for _ in range(len(plan))]
#     for i in range(len(plan)):
#         if mal_function[i]:
#             cur_loc[i] = next_loc[i]
#     print('Malfunction: ', mal_function)
#     print('Current location after update: ', cur_loc)
#     CBS.updateMCP(cur_loc)