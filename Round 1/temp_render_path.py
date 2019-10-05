# coding: utf-8

import numpy as np
import pickle
import sys
import logging

from flatland.utils.rendertools import RenderTool
from flatland.envs.observations import GlobalObsForRailEnv
from flatland.envs.rail_generators import complex_rail_generator
from flatland.envs.schedule_generators import complex_schedule_generator
from flatland.envs.rail_env import RailEnv


def create_env(nr_start_goal=10, nr_extra=2, min_dist=8, max_dist=99999, nr_agent=10, seed=0, render_mode='PIL'):
    env = RailEnv(width=30,
                  height=30,
                  rail_generator=complex_rail_generator(nr_start_goal, nr_extra, min_dist, max_dist, seed),
                  schedule_generator=complex_schedule_generator(),
                  obs_builder_object=GlobalObsForRailEnv(),
                  number_of_agents=nr_agent)
    env_renderer = RenderTool(env, gl=render_mode)
    obs = env.reset()

    return env, env_renderer, obs


class Controller:
    def __init__(self):
        self.idx2node = self.read_file('./config/idx2node_0.pkl')
        self.idx2pose = self.read_file('./config/idx2pos_0.pkl')
        self.node2idx = self.read_file('./config/node2idx_0.pkl')
        self.path_list = self.read_file('./paths.txt')
        self.n_agent = len(self.path_list)

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
        curr_pos = self.idx2pose[self.path_list[agent][time_step]]  # type: tuple
        next_pos = self.idx2pose[self.path_list[agent][time_step+1]]  # type: tuple
        print('curr_pos: ', curr_pos, '\n', next_pos)


if __name__ == '__main__':
    a = Controller()
    a.pos2action(0, 0)

    # with open('./config/idx2node_0.pkl', 'rb') as fin:
    #     temp = pickle.load(file=fin)
    #     print('---------------------')
    #     print(temp)
    #     print(temp[86])
    #     print(temp[105])
