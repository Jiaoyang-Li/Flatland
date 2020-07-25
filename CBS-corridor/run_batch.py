#!/usr/bin/env python

#Compile codes in PythonCBS in folder CBS-corridor with cmake
from libPythonCBS import PythonCBS
# First of all we import the Flatland rail environment
from flatland.envs.rail_env import RailEnv
from flatland.envs.rail_generators import sparse_rail_generator,rail_from_file
from flatland.envs.schedule_generators import sparse_schedule_generator,schedule_from_file
from flatland.envs.malfunction_generators  import malfunction_from_params, MalfunctionParameters,malfunction_from_file

framework = "LNS"  # "LNS" for large neighborhood search or "GPP" for group prioritized planning
f_w = 1
debug = False
timelimit = 240  # unit: seconds
default_group_size = 8 # max number of agents in a group. Suggest 8
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


path = '/mnt/d/Flatland/test-neurips2020-round1-v1/'
import os
import json
results = []
# Uncomment the following lines to continue previous experiments
# with open("summary.txt", 'r') as file:
#     results = json.load(file)  # load existing results

for folder in os.listdir(path):
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

        # Construct the enviornment from file
        test = path + folder + '/' + filename
        env=RailEnv(width=1,height=1,
                    rail_generator=rail_from_file(test),
                    schedule_generator=schedule_from_file(test),
                    malfunction_generator_and_process_data=malfunction_from_file(test),
                    )
        env.reset()

        CBS = PythonCBS(env,framework,"CBSH",timelimit,default_group_size,debug,f_w,
                        corridor_method, chasing ,accept_partial_solution,agent_priority_strategy)
        success = CBS.search()

        # Uncomment the following lines to write the paths to json files
        # plan = CBS.getResult()
        # with open(path + folder + "/" + file_name + ".json", 'w') as file:
        #     json.dump(plan, file)

        # Uncomment the follwing lines to write detailed results to files for performance analysis
        # dirName = "details"
        # if not os.path.exists(dirName):
        #     os.mkdir(dirName)
        # fileName = dirName + "/" + folder + "_" + file_name + "_" + str(env.width) + "x" + str(env.height) + "map_" \
        #            + str(env.get_num_agents()) + "trains" \
        #            + "-w=" + str(f_w) \
        #            + "_groupsize=" + str(default_group_size) \
        #            + "_priority=" + str(agent_priority_strategy) + ".csv"
        # CBS.writeResultsToFile(fileName)

        # Print a summary result and write it to file
        result = CBS.getResultDetail()
        result["instance"] = folder + "_" + file_name
        result["width"] = env.width
        result["height"] = env.height
        result["agents"] = env.get_num_agents()
        result["max_timestep"] = env._max_episode_steps
        print(result)
        results.append(result)
        with open("summary.txt", 'w') as file:
            json.dump(results, file)