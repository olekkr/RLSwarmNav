import time
import numpy as np 

from gym_pybullet_drones.utils.enums import ActionType

import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory
from observation_module import OBS_MODULES
from constants import *
import custom_env

# from cflib.crazyflie.swarm import Swarm
# from cflib.crazyflie.commander import Commander
# from cflib.positioning.motion_commander import MotionCommander
# from cflib.crazyflie.log import LogConfig


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



class Drone:
    def __init__(self, scf):
        self.lc = Commander(scf.cf)
        self.mc = MotionCommander(scf.cf)




        # lg_pos.data_received_cb.add_callback(self._update_state)
        self.obs_mods = [o().cf_init(scf) for o in OBS_MODULES ]
        self.update_obs()
    

    # def _update_state(self, time, data, lg_conf):
    #     self.obs[0] = data["stateEstimate.x"]
    #     self.obs[1] = data["stateEstimate.y"]
    #     self.obs[2] = data["stateEstimate.z"]

    def update_obs(self):
        self.data = np.concat([m.data for m in self.obs_mods])

    def update_act(self, act):
        self.act = act
        

    
def _start(scf, drone:Drone):
    drone.mc.take_off(0.4)
    for o in drone.obs_mods:
        o.start()
    # drone.lg_state.start()
    

def _stop(_, drone:Drone):
    drone.lc.send_notify_setpoint_stop()
    # drone.lc.send_velocity_world_setpoint(0,0,-0.2,0)
    drone.mc.land()
    for o in drone.obs_mods:
        o.stop()
    # drone.lg_state.stop()

def _act(_, drone:Drone):
    if ACTIONTYPE == ActionType.PID:
        drone.lc.send_position_setpoint(*drone.act)
    else: 
        print("not supported action type")
        exit()



class Runtime() : 
    def __init__(self, URIs=URIs):
        self.swarm = Swarm(URIs, factory=CachedCfFactory(rw_cache='./cache'))
        self.swarm.open_links()
        self.droneArg = {}
        self.drones = []

        self.swarm.parallel_safe(self._init_drone, {u:[u] for u in URIs})

        self.policy = custom_env.load_policy()
        self.started = False
        # self.obs = np.zeros_like(self.policy.observation_space)

    def _init_drone(self, scf, uri):
        drone = Drone(scf)
        self.drones.append(drone)
        self.droneArg[uri] = [drone]

    def _start_drones(self):
        self.swarm.reset_estimators() 
        self._collect_obs()
        try:
            self.swarm.parallel_safe(_start, self.droneArg)
            self.started = True
        except Exception as e: 
            print("failed in _start_drones")
            self.stop_drones()
        
    def stop_drones(self):
        self.swarm.parallel(_stop, self.droneArg)
        self.started = False

    def _collect_obs(self): 
        for d in self.drones:
            d.update_obs()
        # FIXME: this is to jank
        # need to insert drone into obs_mods so obs_mods can edit .data
        self.obs = np.stack([d.data for d in self.drones])
        # self.obs = np.stack([np.concat([m.data for m in d.obs_mods]) for d in self.drones])
        
    def _step(self):
        self._collect_obs()
        action, _states = self.policy.predict(self.obs, deterministic=True)
        action = np.concat([action, np.zeros((len(action),1))], axis=1)
        print(f"action: {action}, \nobservation: {self.obs}" )
        for d, a in zip(self.drones, action): 
            d.update_act(a)
        try:
            pass
            self.swarm.parallel_safe(_act, self.droneArg)
        except Exception as e:
            print("failed with exception: in _step()", e, "  stopping.")
            self.stop_drones()


    def run(self, duration):
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
            sleep_t = last_tick-now+target_period
            if sleep_t < 0:
                # TODO: maybe infer max rate, from sleep_t
                i_overstep_period += 1 
                sleep_t = 0 

            time.sleep(sleep_t)
        self.stop_drones()

        if i_overstep_period >0:
            print(f"overstepped {100 * i_overstep_period/i}% of the time")

        self.__del__() 



    def __del__(self):
        # not sure if its the right magic
        print("exiting swarm")
        self.swarm.close_links()
        self.swarm.__exit__(None,None,None)




if __name__ == "__main__":
    cflib.crtp.init_drivers()

    drone = Runtime() 
    drone.run(4)


