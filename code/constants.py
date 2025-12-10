import os 

from gym_pybullet_drones.utils.enums import ActionType, Physics

ACTIONTYPE = ActionType.PID
NUM_AGENTS = 1 
CTRL_FREQ = 30
DEBUG = False
# BUILDNAME = ""
import observation_module as om
OBS_MODULES = [om.XYZPosObs(), om.ZeroObs(54)]

# use different channel for each dongle, for each dongle use different address.
URIs = [ 
    # "radio://0/100/2M/E7E7E7E701",
    "radio://0/100/2M/E7E7E7E702",
    # "radio://0/100/2M/E7E7E7E703",
    # "radio://0/100/2M/E7E7E7E704",
    # "radio://0/100/2M/E7E7E7E705",
    # "radio://0/100/2M/E7E7E7E706",
    # "radio://0/100/2M/E7E7E7E708",
    # "radio://0/100/2M/E7E7E7E709",
    # "radio://0/100/2M/E7E7E7E710"
]
