import numpy as np
import os

from gym_pybullet_drones.envs.BaseRLAviary import BaseRLAviary
from gym_pybullet_drones.utils.enums import DroneModel, Physics, ActionType, ObservationType
from stable_baselines3 import PPO
from gymnasium import spaces


from constants import *


def load_policy(path="results"):
    """
    Load policy interactively

    :param path: Path to policy save directory
    """
    results = sorted(os.listdir(path), reverse=True)
    inpnum = None
    while inpnum is None:
        inp = input(f"""
pick which directory to use ([0]-{len(results)}): \n{list(enumerate(results))}
""")
        try:
            inpnum = int(inp)
        except Exception:
            inpnum = 0 if inp == "" else None

        if inpnum is not None and 0 <= inpnum < len(results):
            break
        else:
            inpnum = None
            print("not valid num")
    print(f"{inpnum} -> {results[inpnum]} chosen.")
    model = PPO.load(os.path.join(path, results[inpnum], "best_model.zip"))
    return model


class CustomAviary(BaseRLAviary):
    """Multi-agent RL problem: leader-follower """

    ###########################################################################

    def __init__(self,
                 drone_model: DroneModel = DroneModel.CF2X,
                 num_drones: int = NUM_AGENTS,
                 neighbourhood_radius: float = np.inf,
                 initial_xyzs=None,
                 initial_rpys=None,
                 physics: Physics = Physics.PYB,
                 pyb_freq: int = 240,
                 ctrl_freq: int = CTRL_FREQ,
                 gui=False,
                 record=False,
                 obs: ObservationType = ObservationType.KIN,
                 ):
        act = ACTIONTYPE
        """Initialization of a multi-agent RL environment.

        Using the generic multi-agent RL superclass.

        Parameters
        ----------
        drone_model : DroneModel, optional
            The desired drone type (detailed in an .urdf file in folder `assets`).
        num_drones : int, optional
            The desired number of drones in the aviary.
        neighbourhood_radius : float, optional
            Radius used to compute the drones' adjacency matrix, in meters.
        initial_xyzs: ndarray | None, optional
            (NUM_DRONES, 3)-shaped array containing the initial XYZ position of the drones.
        initial_rpys: ndarray | None, optional
            (NUM_DRONES, 3)-shaped array containing the initial orientations of the drones (in radians).
        physics : Physics, optional
            The desired implementation of PyBullet physics/custom dynamics.
        pyb_freq : int, optional
            The frequency at which PyBullet steps (a multiple of ctrl_freq).
        ctrl_freq : int, optional
            The frequency at which the environment steps.
        gui : bool, optional
            Whether to use PyBullet's GUI.
        record : bool, optional
            Whether to save a video of the simulation.
        obs : ObservationType, optional
            The type of observation space (kinematic information or vision)
        act : ActionType, optional
            The type of action space (1 or 3D; RPMS, thurst and torques, or waypoint with PID control)

        """
        self.EPISODE_LEN_SEC = 8
        super().__init__(drone_model=drone_model,
                         num_drones=num_drones,
                         neighbourhood_radius=neighbourhood_radius,
                         initial_xyzs=initial_xyzs,
                         initial_rpys=initial_rpys,
                         physics=physics,
                         pyb_freq=pyb_freq,
                         ctrl_freq=ctrl_freq,
                         gui=gui,
                         record=record,
                         obs=obs,
                         act=act
                         )
        self.TARGET_POS = self.INIT_XYZS + \
            np.array([[0, 0, 1/(i+1)] for i in range(num_drones)])

    ###########################################################################

    def _computeReward(self):
        """Computes the current reward value.

        Returns
        -------
        float
            The reward.

        """
        # TODO: needs to take into account dist from other drones.
        states = np.array([self._getDroneStateVector(i)
                          for i in range(self.NUM_DRONES)])
        ret = 0
        for i in range(self.NUM_DRONES):
            ret += max(0, 2 -
                       np.linalg.norm(self.TARGET_POS[i, :]-states[i][0:3])**4)

        return ret

    ###########################################################################

    def _computeTerminated(self):
        """Computes the current done value.

        Returns
        -------
        bool
            Whether the current episode is done.

        """
        # TODO: change
        states = np.array([self._getDroneStateVector(i)
                          for i in range(self.NUM_DRONES)])
        dist = 0
        for i in range(self.NUM_DRONES):
            dist += np.linalg.norm(self.TARGET_POS[i, :]-states[i][0:3])
        if dist < .0001:
            return True
        else:
            return False

    ###########################################################################

    def _computeTruncated(self):
        """Computes the current truncated value.

        Returns
        -------
        bool
            Whether the current episode timed out.

        """
        # TODO: change
        states = np.array([self._getDroneStateVector(i)
                          for i in range(self.NUM_DRONES)])
        for i in range(self.NUM_DRONES):
            # TODO: USE BOX.contains here:
            if (abs(states[i][0]) > 2.0 or abs(states[i][1]) > 2.0 or states[i][2] > 2.0  # Truncate when a drones is too far away
                # Truncate when a drone is too tilted
                        or abs(states[i][7]) > .4 or abs(states[i][8]) > .4
                ):
                return True
        if self.step_counter/self.PYB_FREQ > self.EPISODE_LEN_SEC:  # Trunctate when too much time has elapsed
            return True
        else:
            return False

    ################################################################################

    def _observationSpace(self):
        """Returns the observation space of the environment.

        Returns
        -------

        """
        ############################################################
        # OBS SPACE OF SIZE 12
        # Observation vector ### X        Y        Z       Q1   Q2   Q3   Q4   R       P       Y       VX       VY       VZ       WX       WY       WZ
        # TODO: plug in obs-mod system
        lo = -np.inf
        hi = np.inf
        obs_lower_bound = np.array(
            [[lo, lo, 0, lo, lo, lo, lo, lo, lo, lo, lo, lo] for i in range(self.NUM_DRONES)])
        obs_upper_bound = np.array(
            [[hi, hi, hi, hi, hi, hi, hi, hi, hi, hi, hi, hi] for i in range(self.NUM_DRONES)])
        #### Add action buffer to observation space ################
        act_lo = -1
        act_hi = +1
        for i in range(self.ACTION_BUFFER_SIZE):
            if self.ACT_TYPE in [ActionType.RPM, ActionType.VEL]:
                obs_lower_bound = np.hstack([obs_lower_bound, np.array(
                    [[act_lo, act_lo, act_lo, act_lo] for i in range(self.NUM_DRONES)])])
                obs_upper_bound = np.hstack([obs_upper_bound, np.array(
                    [[act_hi, act_hi, act_hi, act_hi] for i in range(self.NUM_DRONES)])])
            elif self.ACT_TYPE == ActionType.PID:
                obs_lower_bound = np.hstack([obs_lower_bound, np.array(
                    [[act_lo, act_lo, act_lo] for i in range(self.NUM_DRONES)])])
                obs_upper_bound = np.hstack([obs_upper_bound, np.array(
                    [[act_hi, act_hi, act_hi] for i in range(self.NUM_DRONES)])])
            elif self.ACT_TYPE in [ActionType.ONE_D_RPM, ActionType.ONE_D_PID]:
                obs_lower_bound = np.hstack([obs_lower_bound, np.array(
                    [[act_lo] for i in range(self.NUM_DRONES)])])
                obs_upper_bound = np.hstack([obs_upper_bound, np.array(
                    [[act_hi] for i in range(self.NUM_DRONES)])])
        return spaces.Box(low=obs_lower_bound, high=obs_upper_bound, dtype=np.float32)

    def _computeObs(self):
        """Returns the current observation of the environment.
        Returns
        -------
        ndarray
            A Box() of shape (NUM_DRONES,H,W,4) or (NUM_DRONES,12) depending on the observation type.

        """
        if self.OBS_TYPE == ObservationType.RGB:
            if self.step_counter % self.IMG_CAPTURE_FREQ == 0:
                for i in range(self.NUM_DRONES):
                    self.rgb[i], self.dep[i], self.seg[i] = self._getDroneImages(i,
                                                                                 segmentation=False
                                                                                 )
                    #### Printing observation to PNG frames example ############
                    if self.RECORD:
                        self._exportImage(img_type=ImageType.RGB,
                                          img_input=self.rgb[i],
                                          path=self.ONBOARD_IMG_PATH +
                                          "drone_"+str(i),
                                          frame_num=int(
                                              self.step_counter/self.IMG_CAPTURE_FREQ)
                                          )
            return np.array([self.rgb[i] for i in range(self.NUM_DRONES)]).astype('float32')
        elif self.OBS_TYPE == ObservationType.KIN:
            ############################################################
            # OBS SPACE OF SIZE 12
            obs_12 = np.zeros((self.NUM_DRONES, 12))
            for i in range(self.NUM_DRONES):
                # obs = self._clipAndNormalizeState(self._getDroneStateVector(i))
                obs = self._getDroneStateVector(i)
                obs_12[i, :] = np.hstack(
                    [obs[0:3], obs[7:10], obs[10:13], obs[13:16]]).reshape(12,)
            ret = np.array([obs_12[i, :]
                           for i in range(self.NUM_DRONES)]).astype('float32')
            #### Add action buffer to observation #######################
            for i in range(self.ACTION_BUFFER_SIZE):
                ret = np.hstack(
                    [ret, np.array([self.action_buffer[i][j, :] for j in range(self.NUM_DRONES)])])
            return ret
            ############################################################
        else:
            print("[ERROR] in BaseRLAviary._computeObs()")

    def _computeInfo(self):
        """Computes the current info dict(s).

        Unused.

        Returns
        -------
        dict[str, int]
            Dummy value.

        """
        return {"answer": 42}  # Calculated by the Deep Thought supercomputer in 7.5M years
