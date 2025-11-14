from stable_baselines3.common.env_checker import check_env
import stable_baselines3 as sb3 
import pettingzoo
import numpy as np 

from pettingzoo.test import api_test


from monoEnv import *
# TODO: extend droneSim into petting zoo 
class SwarmSim(pettingzoo.AECEnv):
    def __init__(self): 
        pass 



if __name__ == "__main__":
    api_test(SwarmSim)
