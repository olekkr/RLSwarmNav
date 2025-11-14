from stable_baselines3.common.env_checker import check_env
import stable_baselines3 as sb3 
import pettingzoo
import numpy as np 

from pettingzoo.test import api_test


from monoEnv import *
# TODO: extend droneSim into petting zoo 

class SwarmSim(pettingzoo.ParallelEnv):
    metadata = {
        "name": "custom_environment_v0",
    }

    def __init__(self):
        self.monoEnv = DroneSim() 

    def reset(self, seed=None, options=None):
        pass

    def step(self, actions):
        pass

    def render(self):
        pass

    def observation_space(self, agent):
        return self.observation_spaces[agent]

    def action_space(self, agent):
        return self.action_spaces[agent]


if __name__ == "__main__":
    api_test(SwarmSim)
