# coding: utf-8

import numpy as np
import pickle
import time
import logging

from flatland.utils.rendertools import RenderTool
from flatland.envs.observations import GlobalObsForRailEnv
from flatland.envs.rail_generators import complex_rail_generator
from flatland.envs.schedule_generators import complex_schedule_generator
from flatland.envs.rail_env import RailEnv


def create_env(nr_start_goal=10, nr_extra=2, min_dist=8, max_dist=99999, nr_agent=10, seed=0):
    print('Create environment')
    temp_env = RailEnv(width=30,
                       height=30,
                       rail_generator=complex_rail_generator(nr_start_goal, nr_extra, min_dist, max_dist, seed),
                       schedule_generator=complex_schedule_generator(),
                       obs_builder_object=GlobalObsForRailEnv(),
                       number_of_agents=nr_agent)
    temp_obs = temp_env.reset()

    return temp_env, temp_obs


class Controller:
    def __init__(self, in_env):
        print('Controller Initialization')
        self.idx2node = self.read_file('./config/idx2node_0.pkl')
        self.idx2pose = self.read_file('./config/idx2pos_0.pkl')
        self.node2idx = self.read_file('./config/node2idx_0.pkl')
        self.path_list = self.read_file('./path_test.txt')
        self.n_agent = len(self.path_list)
        self.env = in_env
        self.env_renderer = RenderTool(in_env, gl='PILSVG')
        self.max_step = 100
        self.duration = 0.2

    @staticmethod
    def read_file(file_name=None):
        if type(file_name) == str:
            if file_name.split('.')[-1] == 'pkl':
                with open(file_name, 'rb') as fin:
                    return pickle.load(file=fin)
            elif file_name.split('.')[-1] == 'txt':
                with open(file_name, 'r') as fin:
                    path_str = fin.readlines()
                    temp_path = list()
                    for path in path_str:
                        temp_path.append(list(map(int, path.split(',')[:-1])))
                    return temp_path

        else:
            logging.ERROR('Invalid file for reading pickle file.')
            exit(1)

    def pos2action(self, time_step, agent):
        if time_step >= len(self.path_list[agent]) - 1:  # reach goal
            print('Agent {0} reach goal'.format(agent))
            return 2

        else:
            curr_pos, prev_pos = self.idx2node[self.path_list[agent][time_step]]  # type: tuple
            next_pos = self.idx2pose[self.path_list[agent][time_step+1]]  # type: tuple

            # Relative to global frame, type:np.array
            agent_dir = np.subtract(curr_pos, prev_pos) // np.linalg.norm(np.subtract(curr_pos, prev_pos))
            move_dir = np.subtract(next_pos, curr_pos) // np.linalg.norm(np.subtract(next_pos, curr_pos))

            if not np.any(agent_dir + move_dir):  # meet deadend
                print('Agent {0} meets deadend at time step {1}'.format(agent, time_step))
                return 2
            else:
                # Relative to agent frame
                out_dir = (agent_dir[0]*move_dir[0] + agent_dir[1]*move_dir[1],
                           -agent_dir[1]*move_dir[0] + agent_dir[0]*move_dir[1])

                # print('out_dir: ', out_dir)
                if out_dir == (0, 0):  # stay
                    return 4
                elif out_dir == (0, 1):  # move left
                    return 1
                elif out_dir == (1, 0):  # move forward
                    return 2
                elif out_dir == (0, -1):  # move right
                    return 3

    def get_actions(self, time_step):
        _actions = {}
        for _idx in range(self.n_agent):
            # if _idx == 9:
            #     _actions[_idx] = self.pos2action(time_step, _idx)
            # else:
            #     _actions[_idx] = 4
            _actions[_idx] = self.pos2action(time_step, _idx)
        return _actions

    def render_actions(self):
        print('Start Rendering')
        for step in range(self.max_step):
            print('step: ', step)
            out_actions = self.get_actions(time_step=step)
            _obs, _all_rewards, _done, _ = self.env.step(out_actions)  # take one step with the provided actions.
            self.env_renderer.render_env(show=True, frames=False, show_observations=False)
            time.sleep(self.duration)
            input('Press enter to move on')


if __name__ == '__main__':
    env, obs = create_env()
    a = Controller(in_env=env)
    a.render_actions()
    # for i in range(len(a.path_list[0])):
    #     print('time_step: {0}, action: {1}'.format(i, a.pos2action(time_step=i, agent=0)))
