

import os
import time
from datetime import datetime
import argparse
import gymnasium as gym
import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.callbacks import EvalCallback, StopTrainingOnRewardThreshold
from stable_baselines3.common.evaluation import evaluate_policy

from gym_pybullet_drones.utils.Logger import Logger
from gym_pybullet_drones.utils.utils import sync, str2bool
from gym_pybullet_drones.utils.enums import ObservationType, ActionType

import CustomEnv 


NUM_AGENTS = 2 

results = sorted(os.listdir("results"), reverse=True)
print(list(zip(range(len(results)))), results)

#TODO: load a specific model by specifying index.
#TODO: extract info on num of agents
model = PPO.load(os.path.join("results", results[0], "best_model.zip"))


test_env = CustomEnv.CustomAviary(gui=True,
                                num_drones=NUM_AGENTS)

# test_env_nogui = CustomEnv.CustomAviary(num_drones=NUM_AGENTS)

# mean_reward, std_reward = evaluate_policy(model,
#                                               test_env_nogui,
#                                               n_eval_episodes=10)
# print(mean_reward, std_reward)



logger = Logger(logging_freq_hz=int(test_env.CTRL_FREQ),
            num_drones=NUM_AGENTS,
            # output_folder=output_folder,
            colab=False
            )



obs, info = test_env.reset(seed=42, options={})
start = time.time()
for i in range((test_env.EPISODE_LEN_SEC+2)*test_env.CTRL_FREQ):
    action, _states = model.predict(obs,
                                    deterministic=True
                                    )
    obs, reward, terminated, truncated, info = test_env.step(action)
    obs2 = obs.squeeze()
    act2 = action.squeeze()
    print("Obs:", obs, "\tAction", action, "\tReward:", reward, "\tTerminated:", terminated, "\tTruncated:", truncated)
    # for d in range(NUM_AGENTS):
    #     logger.log(drone=d,
    #         timestamp=i/test_env.CTRL_FREQ,
    #         state=np.hstack([obs2[d][0:3],
    #                             np.zeros(4),
    #                             obs2[d][3:15],
    #                             act2[d]
    #                             ]),
    #         control=np.zeros(12)
    #         )
    test_env.render()
    print(terminated)
    sync(i, start, test_env.CTRL_TIMESTEP)
    if terminated or truncated:
        obs = test_env.reset(seed=42, options={})
test_env.close()