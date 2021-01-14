#!/usr/bin/env python

# Compile codes in PythonCBS in folder CBS-corridor with cmake and import PythonCBS class
from libPythonCBS import PythonCBS

# Import the Flatland rail environment
from flatland.envs.rail_env import RailEnv
from flatland.envs.rail_generators import sparse_rail_generator,rail_from_file
from flatland.envs.schedule_generators import sparse_schedule_generator,schedule_from_file
from flatland.envs.malfunction_generators  import malfunction_from_params, MalfunctionParameters,malfunction_from_file,ParamMalfunctionGen
from flatland.core.env_observation_builder import DummyObservationBuilder
from flatland.utils.rendertools import RenderTool, AgentRenderVariant
import time, glob

env_renderer_enable = True

#####################################################################
# malfunction parameters
#####################################################################
malfunction_rate = 1/200          # fraction number, probability of having a stop.
min_duration = 20
max_duration = 50


stochastic_data = MalfunctionParameters(malfunction_rate=0/600,  # Rate of malfunction occurence
                                        min_duration=30,  # Minimal duration of malfunction
                                        max_duration=30  # Max duration of malfunction
                                        )


rail_generator = sparse_rail_generator(max_num_cities=5,
                                       seed=10,
                                       grid_mode=False,
                                       max_rails_between_cities=1,
                                       max_rails_in_city=3,
                                       )
# Different agent types (trains) with different speeds.
speed_ration_map = {1.: 1,  # Fast passenger train
                    1. / 2.: 0,  # Fast freight train
                    1. / 3.: 0,  # Slow commuter train
                    1. / 4.: 0}  # Slow freight train

# We can now initiate the schedule generator with the given speed profiles

schedule_generator = sparse_schedule_generator(speed_ration_map)



#####################################################################
# Initialize flatland environment
#####################################################################

local_env = RailEnv(width=40,
                    height=30,
                    rail_generator=rail_generator,
                    schedule_generator=schedule_generator,
                    number_of_agents=1,
                    obs_builder_object=DummyObservationBuilder(),
                    #malfunction_generator_and_process_data=malfunction_from_params(stochastic_data),
                    malfunction_generator=ParamMalfunctionGen(stochastic_data),
                    remove_agents_at_target=True,
                    random_seed=100)

local_env.reset()

#####################################################################
# Initialize Mapf-solver
#####################################################################
framework = "LNS"  # "LNS" for large neighborhood search
default_group_size = 5 # max number of agents in a group.
max_iterations = 1000
stop_threshold = 10
agent_priority_strategy = 3
neighbor_generation_strategy = 3
debug = False
time_limit =200
replan = True

solver = PythonCBS(local_env, framework, time_limit, default_group_size, debug, replan,stop_threshold,agent_priority_strategy,neighbor_generation_strategy)
solver.search(1.1, max_iterations)
solver.buildMCP()

#####################################################################
# Show the flatland visualization, for debugging
#####################################################################

if env_renderer_enable:
    env_renderer = RenderTool(local_env, screen_height=local_env.height * 50,
                              screen_width=local_env.width*50,show_debug=False)
    env_renderer.render_env(show=True, show_observations=False, show_predictions=False)

steps=0
while True:
    #####################################################################
    # Simulation main loop
    #####################################################################

    # Get action dictionary from mapf solver.
    action =  solver.getActions(local_env, steps, 3.0)

    observation, all_rewards, done, info = local_env.step(action)

    if env_renderer_enable:
        env_renderer.render_env(show=True, show_observations=False, show_predictions=False)
        time.sleep(0.5)


    steps += 1
    if done['__all__']:
        solver.clearMCP()
        break



