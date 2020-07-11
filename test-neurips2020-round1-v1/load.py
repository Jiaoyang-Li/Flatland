#!/usr/bin/env python3

# First of all we import the Flatland rail environment
from flatland.envs.rail_env import RailEnv
from flatland.envs.rail_generators import rail_from_file
from flatland.envs.schedule_generators import schedule_from_file
from flatland.envs.malfunction_generators  import malfunction_from_file


path = './'  # change this line to your test files location if you put the python code in a differnt folder

import os
import json
for folder in os.listdir(path):
    if os.path.isfile(path + folder):
        continue
    for filename in os.listdir(path+folder):
        file_name,extension = os.path.splitext(filename)
        if extension != '.pkl' and extension != '.mpk':
            continue

        print("\n\n Instance " + folder + '/' + filename)

        # Construct the enviornment from file
        test = path + folder + '/' + filename
        env=RailEnv(width=1,height=1,
                    rail_generator=rail_from_file(test),
                    schedule_generator=schedule_from_file(test),
                    malfunction_generator_and_process_data=malfunction_from_file(test),
            )

        # Read paths from file
        with open(path + folder + "/" + file_name + ".json", 'r') as file:
            paths = json.load(file)  # load paths

        for p in paths:
            print(p)