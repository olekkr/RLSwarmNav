import os
import time
from datetime import datetime
import argparse
import gymnasium as gym
import numpy as np
import stable_baselines3
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.callbacks import EvalCallback, CallbackList, CheckpointCallback
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.monitor import Monitor

from gym_pybullet_drones.utils.Logger import Logger
from gym_pybullet_drones.utils.utils import sync

from constants import * 
import custom_env

# tune n_envs to available CPUs
N_ENVS = min(6, max(1, (os.cpu_count() or 2) - 1))

train_env = make_vec_env(
    custom_env.CustomAviary,
    env_kwargs={"num_drones": NUM_AGENTS, "gui": False, "record": False},
    n_envs=N_ENVS,
    seed=0)

# wrap eval env in Monitor to reduce logging overhead
from stable_baselines3.common.monitor import Monitor
eval_env = Monitor(custom_env.CustomAviary(num_drones=NUM_AGENTS, gui=False))

device = "cuda" if torch.cuda.is_available() else "cpu"
print(f"[INFO] Using device: {device}")

# Model hyperparameters
model_hyperparams = {
    'policy': 'MlpPolicy',
    'n_steps': 1024,
    'batch_size': 256,
    'n_epochs': 10,
    'learning_rate': 3e-4,
    'device': device,
    'num_envs': N_ENVS,
    'total_timesteps': 200_000,
    'eval_freq': 20_000,
    'n_eval_episodes': 5,
    'num_agents': NUM_AGENTS,
    'action_type': str(ACTIONTYPE),
    'control_frequency': CTRL_FREQ,
    'max_speed': MAX_SPEED,
    'episode_length_sec': EPISODE_LEN_SEC
}


custom_name = input("enter custom name (default: save-{timestamp} )\n") + "_"
if custom_name == "_":
    filename = os.path.join("results", 'save-'+datetime.now().strftime("%Y.%m.%d.%H:%M:%S"))
else: 
    filename = os.path.join("results", custom_name+datetime.now().strftime("%Y.%m.%d.%H:%M:%S"))

os.makedirs(filename, exist_ok=True)

# set device and (optionally) smaller n_steps for faster updates
model = PPO('MlpPolicy',
            train_env,
            verbose=1, 
            device=device,
            n_steps=1024,
            batch_size=128,
            n_epochs=10,
            tensorboard_log=filename,)  # Enable TensorBoard logging


# Save hyperparameters to file
with open(os.path.join(filename, 'hyperparameters.txt'), 'w') as f:
    f.write("Training Hyperparameters\n")
    f.write("=" * 50 + "\n\n")
    for key, value in model_hyperparams.items():
        f.write(f"{key}: {value}\n")
    f.write("\n" + "=" * 50 + "\n")

print(f"[INFO] Training configuration saved to {filename}/hyperparameters.txt")



# less frequent evals and avoid saving every eval (set best_model_save_path=None to skip frequent model saves)
eval_callback = CallbackList([
    EvalCallback(eval_env,
                             callback_on_new_best=None,
                             callback_after_eval=None,
                             verbose=1,
                             best_model_save_path=filename,  
                             log_path=filename+'/',
                             eval_freq=20_000,           # run evaluation less often
                             n_eval_episodes=5,
                             deterministic=True,
                             render=False),
    CheckpointCallback(save_freq=200_000,
                        save_path=filename,
                        name_prefix='rl_model_checkpoint')
    ])

print("training...", N_ENVS)
model.learn(total_timesteps=int(50_000_000),
                callback=eval_callback,
                log_interval=100,  # log every 100 policy update calls for frequent TensorBoard data
                progress_bar=True,
                tb_log_name="PPO"
                )


#### Save the model ########################################


model_save_location = filename+'/final_model.zip'
print(f"saving to: {model_save_location}")
model.save(model_save_location)



