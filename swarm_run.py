"""Swarm runtime: wrap Crazyflie swarm connections and run a policy.

This module provides a lightweight `Drone` wrapper around a Crazyflie
connection and a `Runtime` that runs a control policy across a swarm.
"""

import time
import numpy as np 

from gym_pybullet_drones.utils.enums import ActionType

import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory
from observation_module import ObsFactory
from constants import *

import custom_env


# from cflib.crazyflie.swarm import Swarm
# from cflib.crazyflie.commander import Commander
# from cflib.positioning.motion_commander import MotionCommander
# from cflib.crazyflie.log import LogConfig


# Use mock Crazyflie classes when running in DEBUG mode (no real hardware)
if DEBUG:
    from mock_crazyflie import MockSwarm as Swarm 
    from mock_crazyflie import MockCommander as Commander 
    from mock_crazyflie import MockMotionCommander as MotionCommander
    from mock_crazyflie import MockLogConfig as LogConfig
else:
    from cflib.crazyflie.swarm import Swarm
    from cflib.crazyflie.commander import Commander
    from cflib.positioning.motion_commander import MotionCommander
    from cflib.crazyflie.log import LogConfig

OBS_FACTORY = ObsFactory(OBS_SIGNATURE)

class Drone:
    """Per-connection wrapper providing command interfaces and observations.

    """
    def __init__(self, scf):
        # Command interfaces for this drone instance
        self.lc = Commander(scf.cf)
        self.mc = MotionCommander(scf.cf)

        # Initialize observation modules attached to the Crazyflie
        # self.obs_mods = [o().cf_init(scf) for o in OBS_MODULES ]
        self.obs_container = OBS_FACTORY.generate().cf_init(scf)

        # Populate initial observation buffer
        # self.update_obs() # FIXME: does this work?

    def update_obs(self):
        """Pull and combine observations from all attached modules.

        Result is stored in `self.data` as a single NumPy array per drone.
        """
        # Concatenate data from all observation modules into one array
        self.data = self.obs_container.get_data()

    def update_act(self, act):
        self.act = act
        

######################## DRONE callbacks ########################
# functions used via swarm.parallel to send commands to the drones.
# All helpers are intedended to be called via the swarm parallel API
def _start(scf, drone):
    """Start per-drone observation modules and perform takeoff.
    """
    # 
    drone.obs_container.start()
    # for o in drone.obs_mods:
    #     o.start()

    drone.mc.take_off(0.5) 

def _stop(_, drone):
    """Stop motion, observation modules and land for a single drone.
    """
    drone.lc.send_notify_setpoint_stop()
    drone.mc.land()
    drone.obs_container.stop()
    # for o in drone.obs_mods:
    #     o.stop()

def _act(_, drone):
    """Apply stored action to the drone using the configured interface.
    """
    if ACTIONTYPE == ActionType.VEL:
        print(drone.act[0] * drone.act[3], drone.act[1] * drone.act[2], drone.act[2] * drone.act[3], 0)
        s = drone.act[3] * 0.25 # plug into global speed limit
        drone.lc.send_velocity_world_setpoint(
            drone.act[0] * s,
            drone.act[1] * s,
            drone.act[2] * s, 
            0
            )
    else: 
        print("not supported action type")
        exit()
####################################################################


class Runtime() : 
    """Run a control policy across a Crazyflie swarm.

    Creates `Drone` wrappers for each URI, starts/stops the swarm,
    collects observations, invokes the policy, and dispatches actions.
    Runs the control loop via the `run` method.
    """

    def __init__(self, URIs=URIs):
        self.swarm = Swarm(URIs, factory=CachedCfFactory(rw_cache='./cache'))
        self.swarm.open_links()
        self.droneArg = {}
        self.drones = []

        # Initialize drone wrappers in parallel
        self.swarm.parallel_safe(self._init_drone, {u:[u] for u in URIs})

        # Load control policy (e.g., from custom environment / RL agent)
        self.policy = custom_env.load_policy()
        self.started = False

    def _init_drone(self, scf, uri):
        """Create and register a `Drone` wrapper for a swarm connection.
        """
        drone = Drone(scf)
        self.drones.append(drone)
        self.droneArg[uri] = [drone]

    def _start_drones(self):
        """Reset estimators, prime observations, then start all drones.
        """
        self.swarm.reset_estimators() 
        self._collect_obs()
        try:
            self.swarm.parallel_safe(_start, self.droneArg)
            self.started = True
        except Exception as e: 
            print("failed in _start_drones")
            self.stop_drones()
        
    def stop_drones(self):
        """Request all drones to stop and land via the swarm API."""
        self.swarm.parallel(_stop, self.droneArg)
        self.started = False

    def _collect_obs(self): 
        """Update observations for every drone and stack into `self.obs`.

        The stacked array has shape (n_drones, obs_dim)
        """
        for d in self.drones:
            d.update_obs()
        self.obs = np.stack([d.data for d in self.drones])
        
    def _step(self):
        """Perform one control loop step: observe, predict, and dispatch.
        Observcations are collected from all drones;
        The policy's `predict` is called; 
        The resulting actions are sent to each drone.
        """
        self._collect_obs()
        action, _states = self.policy.predict(self.obs, deterministic=True)
        print(f"action: \n{action}, \nobservation: \n{self.obs}" )
        for d, a in zip(self.drones, action): 
            d.update_act(a)
        try:
            self.swarm.parallel_safe(_act, self.droneArg)
        except Exception as e:
            print("failed with exception: in _step()", e, "  stopping.")
            self.stop_drones()


    def run(self, duration):
        """Run the control loop for `duration` seconds.

        Runs `_step` at `CTRL_FREQ` Hz for the specified duration.
        """
        if not self.started:
            self._start_drones()

        start = time.perf_counter()

        target_period = 1/CTRL_FREQ 
        i = 0 
        i_overstep_period = 0
        while True: 
            last_tick = time.perf_counter()
            
            i += 1
            self._step()
            now = time.perf_counter()
            if now - start > duration:
                break
            sleep_t = target_period-(last_tick-now)
            if sleep_t < 0:
                i_overstep_period += 1 
                sleep_t = 0 

            time.sleep(sleep_t)
        self.stop_drones()

        if i_overstep_period >0:
            print(f"overstepped {100 * i_overstep_period/i}% of the time")

        self.__del__() 



    def __del__(self):
        """Attempt to gracefully close swarm resources on object deletion."""
        print("exiting swarm")
        self.swarm.close_links()
        self.swarm.__exit__(None,None,None)




if __name__ == "__main__":
    cflib.crtp.init_drivers()

    np.set_printoptions(precision=3, sign=" ", suppress=True)

    drone = Runtime() 
    drone.run(30)



