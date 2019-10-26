import numpy as np
import time
from flatland.envs.rail_generators import complex_rail_generator
from flatland.envs.schedule_generators import complex_schedule_generator
from flatland.envs.rail_env import RailEnv, RailEnvActions
from flatland.utils.rendertools import RenderTool
from flatland.envs.agent_utils import RailAgentStatus, EnvAgent

NUMBER_OF_AGENTS = 10


def my_controller():
    """
    You are supposed to write this controller
    """
    temp_action = {}
    for _idx in range(NUMBER_OF_AGENTS):
        temp_action[_idx] = RailEnvActions(np.random.randint(0, 5))
    return temp_action


if __name__ == '__main__':
    env = RailEnv(
                width=20,
                height=20,
                rail_generator=complex_rail_generator(
                                        nr_start_goal=10,
                                        nr_extra=1,
                                        min_dist=8,
                                        max_dist=99999,
                                        seed=20),
                schedule_generator=complex_schedule_generator(),
                number_of_agents=NUMBER_OF_AGENTS)

    for _agent in env.agents:
        print(_agent.speed_data)

    env_renderer = RenderTool(env)

    for step in range(100):
        _action = my_controller()
        obs, all_rewards, done, _ = env.step(_action)
        print("Rewards: {}, [done={}]".format(all_rewards, done))
        env_renderer.render_env(show=True, frames=False, show_observations=False)
        time.sleep(1.0)
        input('Press enter to move on')  # un-command this line if you want to show step-by-step motion

