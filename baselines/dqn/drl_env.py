import os
import sys

import numpy as np
import gym
import random

# Add current working directory to path so we can import
from environment import VRPEnvironment
from solver import solve_static_vrptw
from baselines.strategies._strategies import _filter_instance
from utils import get_request_features

_BIG_NUMBER = int(1e9)


class DRLEnv(gym.Env):
    """
    Environment for learning to take actions with DRL
    """

    def __init__(self, config, **kwargs):
        self.config = config["training"]
        self.features_k_nearest = config['model']['k_nearest']
        self.num_of_feats = (8 + 1 + self.features_k_nearest * 9) + (4 + 7)
        self.counter = 0
        self.seed = 0
        self.instance = None
        self.instance_env = None
        self.rng = None

        self.action_set = []
        self.last_action = None
        self.instance_observation = None
        self.instance_static_info = None
        self.request_features = None
        self.global_features = None
        self.epoch_tlim = self.config['epoch_tlim']

        self.action_space = gym.spaces.Discrete(2)
        self.observation_space = gym.spaces.Box(shape=(5,), low=-_BIG_NUMBER, high=_BIG_NUMBER, dtype=np.float64)

        self.solver_tmp_dir = os.path.join(self.config['ckpt_dir'], "tmp")
        os.makedirs(self.solver_tmp_dir, exist_ok=True)

    def make_state(self):
        return np.concatenate((self.request_features[self.counter], self.global_features))

    def reset(self, instance=None):
        self.counter = 0
        if instance is not None:
            self.instance = instance
        assert instance is not None
        self.instance_env = VRPEnvironment(seed=self.seed, instance=instance, epoch_tlim=self.epoch_tlim, is_static=False)
        self.instance_observation, self.instance_static_info = self.instance_env.reset()
        self.max_number_of_actions = self.instance_observation['epoch_instance']['coords'].shape[0]
        self.request_features, self.global_features = get_request_features(self.instance_observation, self.instance_static_info, self.features_k_nearest)
        self.action_set = []

        return self.make_state(), {}

    def step(self, action):
        self.action_set.append(action)
        self.counter += 1
        self.seed += 1
        reward = 0

        if self.counter == self.max_number_of_actions:  # TODO:CHECK
            self.instance_observation, reward, done, _ = self.deploy_actions(self.action_set)
            if done:
                return None, reward, True, {}
            self.request_features, self.global_features = get_request_features(self.instance_observation, self.instance_static_info, self.features_k_nearest)
            self.action_set = []
            self.counter = 0
            self.max_number_of_actions = self.instance_observation['epoch_instance']['coords'].shape[0]
        
        return self.make_state(), reward, False, {}
    
    def deploy_actions(self, action_set):

        mask = self.instance_env.epoch_instance['must_dispatch'] | (np.array(action_set) == 0)
        mask[0] = True  # Depot always included in scheduling

        epoch_instance_dispatch = _filter_instance(self.instance_observation['epoch_instance'], mask)

        # get route with solver
        solutions = list(solve_static_vrptw(
            epoch_instance_dispatch, time_limit=self.epoch_tlim, seed=self.seed, 
            tmp_dir=self.solver_tmp_dir))

        assert len(solutions) > 0, f"No solution found during epoch {self.instance_observation['current_epoch']}"
        epoch_solution, cost = solutions[-1]

        # Map HGS solution to indices of corresponding requests
        epoch_solution = [epoch_instance_dispatch['request_idx'][route] for route in epoch_solution]

        # Submit solution to environment and return results
        return self.instance_env.step(epoch_solution)
