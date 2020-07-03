#!/usr/bin/env python

#Compile codes in PythonCBS in folder CBS-corridor with cmake

import numpy as np
import time
from flatland.envs.observations import GlobalObsForRailEnv
# First of all we import the Flatland rail environment
from flatland.envs.rail_env import RailEnv
from flatland.envs.rail_generators import sparse_rail_generator,rail_from_file
from flatland.envs.schedule_generators import sparse_schedule_generator,schedule_from_file
from flatland.envs.malfunction_generators  import malfunction_from_params, MalfunctionParameters,malfunction_from_file

# We also include a renderer because we want to visualize what is going on in the environment
from flatland.utils.rendertools import RenderTool, AgentRenderVariant
from libPythonCBS import PythonCBS


path = '/mnt/d/Flatland/CBS-corridor/test-neurips2020-round1-v1/'

f_w = 1
debug = False
k = 1
timelimit = 240  # unit: seconds
default_group_size = 16 # max number of agents in a group
corridor_method = "trainCorridor1" # or "corridor2" or ""
accept_partial_solution = True
agent_priority_strategy = 0  #  choose a number between 0 and 5
#                               0: keep the original ordering
#                               1: prefer max speed then max distance
#                               2: prefer min speed then max distance
#                               3: prefer max speed then min distance
#                               4: prefer min speed then min distance
#                               5: prefer different start locations then max speed then max distance

import os
results = []
for folder in os.listdir(path):
    if os.path.isfile(path + folder):
        continue
    for filename in os.listdir(path+folder):
        file_name,extension = os.path.splitext(filename)
        if extension != '.pkl' and extension != '.mpk':
            continue
        test = path + folder + '/' + filename
        print("Instance " + folder + '/' + filename)
        # Construct the enviornment with the given observation, generataors, predictors, and stochastic data
        env=RailEnv(width=1,height=1,
                    rail_generator=rail_from_file(test),
                    schedule_generator=schedule_from_file(test),
                    malfunction_generator_and_process_data=malfunction_from_file(test),
                    )
        # Initiate the renderer
        env.reset()

        CBS = PythonCBS(env,"CBSH",k,timelimit,default_group_size,debug,f_w,
                        corridor_method,accept_partial_solution,agent_priority_strategy)
        success = CBS.search()

        # write results to files for performance analysis
        fileName = folder + "_" + file_name + "_" + str(env.width) + "x" + str(env.height) + "map_" \
                   + str(env.get_num_agents()) + "trains" \
                   + "_groupsize=" + str(default_group_size) \
                   + "_priority=" + str(agent_priority_strategy) + ".csv"
        CBS.writeResultsToFile(fileName)

        result = CBS.getResultDetail()
        result["instance"] = folder + "_" + file_name
        result["width"] = env.width
        result["height"] = env.height
        result["#agents"] = env.get_num_agents()
        print(result)
        results.append(result)
        import json
        with open("summary.txt", 'w') as file:
            json.dump(results, file)
