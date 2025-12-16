
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

from constants import * 
import custom_env 



train_env = make_vec_env(
    custom_env.CustomAviary, 
    env_kwargs={"num_drones":NUM_AGENTS}, 
    n_envs=6, 
    seed=0)
eval_env = custom_env.CustomAviary(num_drones=NUM_AGENTS)


print('[INFO] Action space:', train_env.action_space)
print('[INFO] Observation space:', train_env.observation_space)



#### Train the model #######################################
model = PPO('MlpPolicy',
            train_env,
            # tensorboard_log=filename+'/tb/',
            verbose=1)

filename = os.path.join("results", 'save-'+datetime.now().strftime("%m.%d.%Y_%H.%M.%S"))
eval_callback = EvalCallback(eval_env,
                                # callback_on_new_best=callback_on_best,
                                verbose=1,
                                best_model_save_path=filename+'/',
                                log_path=filename+'/',
                                eval_freq=int(1000),
                                deterministic=True,
                                render=False)

print("training...")
model.learn(total_timesteps=int(1e6),
                callback=eval_callback,
                log_interval=10000)



#### Save the model ########################################

# TODO: add info on num of agents
model_save_location = filename+'/final_model.zip'
print(f"saving to: {model_save_location}")
model.save(model_save_location)

