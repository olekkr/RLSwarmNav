import os 

from gym_pybullet_drones.utils.enums import ActionType, Physics
import numpy as np
import util

np.set_printoptions(precision=3, sign=" ", suppress=True)

# use different channel for each dongle, for each dongle use different address.
URIs = [
    "radio://0/100/2M/E7E7E7E701",
    "radio://0/100/2M/E7E7E7E702",
    "radio://0/100/2M/E7E7E7E703",
    "radio://0/100/2M/E7E7E7E704",
    # "radio://0/100/2M/E7E7E7E705",
    # "radio://0/100/2M/E7E7E7E706",
    # "radio://0/100/2M/E7E7E7E707",
    # "radio://0/100/2M/E7E7E7E708",
    # "radio://0/100/2M/E7E7E7E709",
    # "radio://0/100/2M/E7E7E7E710"
]

OBS_SIGNATURE = [
    ("PosObs",{}),
    # ("QUATObs",{}),
    ("RPYObs",{}),
    ("VelObs",{}),
    ("AngRateObs",{}),
    # ("TargetPosObs", {"position": None}),
    # ("RelTargetPos", {"position": None}),
    # ("ZeroObs", {"size":36}),
    # ("RelDronePos", {"size": NUM_AGENTS*3})
]


ACTIONTYPE = ActionType.VEL
NUM_AGENTS = len(URIs) 
CTRL_FREQ = 30
DEBUG = False
BOUNDING_BOX = util.Box( [-2,-2,0], [4,4,2])
EPISODE_LEN_SEC = 120
MAX_SPEED = 0.25  # m/s

#####################################
# pos Swap test: 
# in a circle at height 0.5, radius 1.5
r = 1.5
INITIAL_XYZS = np.array([[r*np.cos(2*np.pi*i/NUM_AGENTS), r*np.sin(2*np.pi*i/NUM_AGENTS), 0.5] for i in range(NUM_AGENTS)])
permutaions = [(i + NUM_AGENTS//2)%NUM_AGENTS for i in range(NUM_AGENTS)]
TARGET_POS = np.array([INITIAL_XYZS[permutaions[i]] for i in range(NUM_AGENTS)])
#####################################



