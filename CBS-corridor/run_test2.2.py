#!/usr/bin/env python

#Compile codes in PythonCBS in folder CBS-corridor with cmake

import numpy as np
import time
from flatland.envs.observations import GlobalObsForRailEnv
# First of all we import the Flatland rail environment
from flatland.envs.rail_env import RailEnv
from flatland.envs.rail_generators import sparse_rail_generator
from flatland.envs.schedule_generators import sparse_schedule_generator
from flatland.envs.malfunction_generators  import malfunction_from_params, MalfunctionParameters

# We also include a renderer because we want to visualize what is going on in the environment
from flatland.utils.rendertools import RenderTool, AgentRenderVariant
from libPythonCBS import PythonCBS


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

# Construct the enviornment with the given observation, generataors, predictors, and stochastic data
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

f_w = 1
debug = True
k = 1
timelimit = 10
corridor_method = "trainCorridor1" # or "corridor2" or ""
CBS = PythonCBS(env,"ICBS",k,timelimit,debug,f_w,corridor_method)
success = CBS.search()
plan = CBS.getResult()

print(plan)