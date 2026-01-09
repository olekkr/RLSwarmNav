

import time
from datetime import datetime
# import argparse
import gymnasium as gym
import numpy as np
from stable_baselines3.common.evaluation import evaluate_policy

from gym_pybullet_drones.utils.Logger import Logger
from gym_pybullet_drones.utils.utils import sync, str2bool
from gym_pybullet_drones.utils.enums import ObservationType, ActionType

from constants import *
import custom_env 




policy = custom_env.load_policy()

test_env = custom_env.CustomAviary(
    gui=True,
    num_drones=NUM_AGENTS)

test_env_nogui = custom_env.CustomAviary(num_drones=NUM_AGENTS)

mean_reward, std_reward = evaluate_policy(policy,
                                              test_env_nogui,
                                              n_eval_episodes=10)
print(mean_reward, std_reward)



# logger = Logger(logging_freq_hz=int(test_env.CTRL_FREQ),
#             num_drones=NUM_AGENTS,
#             # output_folder=output_folder,
#             colab=False
#             )



obs, info = test_env.reset(seed=1, options={})
start = time.time()
for i in range(3* (test_env.EPISODE_LEN_SEC+2)*test_env.CTRL_FREQ):
    action, _states = policy.predict(obs,
                                    deterministic=True
                                    )

    # FIXME: test:
    # action = action* np.array( [
    #                           [1,1,1],
    #                           [1,1,-1],
    #                           [1,1,-1],
    #     ])

    obs, reward, terminated, truncated, info = test_env.step(action)
    print(obs,reward, terminated, truncated)
    obs2 = obs.squeeze()
    act2 = action.squeeze()
    # print("\tAction", action, "\tReward:", reward, "\tTerminated:", terminated, "\tTruncated:", truncated)
    
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
    sync(i, start, test_env.CTRL_TIMESTEP)
    if terminated or truncated:
        print(f"Terminated: {terminated}, Truncated: {truncated} at step {i}")
        obs, _ = test_env.reset(seed=42, options={})

print(f"obs: {obs} \n target pos: {test_env.TARGET_POS}")
test_env.close()
