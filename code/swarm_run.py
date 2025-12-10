import time
import numpy as np 

from gym_pybullet_drones.utils.enums import ActionType

import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory

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

        #need:  X        Y        Z       Q1   Q2   Q3   Q4   R       P       Y       VX       VY       VZ       WX       WY       WZ
        # state estimate logging


        # Position observation module: 
        lg_pos = LogConfig("stateEstimate", 1000/CTRL_FREQ)
        lg_pos.add_variable("stateEstimate.x", "float")
        lg_pos.add_variable("stateEstimate.y", "float")
        lg_pos.add_variable("stateEstimate.z", "float")
        scf.cf.log.add_config(lg_pos)
        self.lg_state = lg_pos

        # lg_state.add_variable("stateEstimate.qw", "float")
        # lg_state.add_variable("stateEstimate.qx", "float")
        # lg_state.add_variable("stateEstimate.qy", "float")
        # lg_state.add_variable("stateEstimate.qz", "float")
        
        
        # lg_state.add_variable("stateEstimate.roll", "float")
        # lg_state.add_variable("stateEstimate.pitch", "float")
        # lg_state.add_variable("stateEstimate.yaw", "float")

        # lg_state.add_variable("stateEstimate.vx", "float")
        # lg_state.add_variable("stateEstimate.vy", "float")
        # lg_state.add_variable("stateEstimate.vz", "float")

        # lg_state2 = LogConfig("stateEstimateZ", 1000/CTRL_FREQ)
        # lg_state2.add_variable("stateEstimateZ.rateRoll", "float")
        # lg_state2.add_variable("stateEstimateZ.ratePitch", "float")
        # lg_state2.add_variable("stateEstimateZ.rateYaw", "float")


        # scf.cf.log.add_config(lg_state2)


        lg_pos.data_received_cb.add_callback(self._update_state)
        self.obs = np.zeros(57)

    def _update_state(self, stamp, data, lg_conf):
        # FIXME:: change 
        self.obs[0] = data["stateEstimate.x"]
        self.obs[1] = data["stateEstimate.y"]
        self.obs[2] = data["stateEstimate.z"]


    def update_act(self, act):
        self.act = act

    
def _start(scf, drone:Drone):
    drone.mc.take_off(0.4)
    drone.lg_state.start()
    

def _stop(_, drone:Drone):
    drone.lc.send_notify_setpoint_stop()
    # drone.lc.send_velocity_world_setpoint(0,0,-0.2,0)
    drone.mc.land()
    drone.lg_state.stop()

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
        self.obs = np.zeros_like(self.policy.observation_space)

    def _init_drone(self, scf, uri):
        drone = Drone(scf)
        self.drones.append(drone)
        self.droneArg[uri] = [drone]

    def _start_drones(self):
        self.swarm.reset_estimators() 
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
        self.obs = np.stack([np.array(d.obs) for d in self.drones],axis=0)
        
    def _step(self):
        self._collect_obs()
        print(self.obs)
        action, _states = self.policy.predict(self.obs, deterministic=True)
        action = np.concat([action, np.zeros((len(action),1))], axis=1)
        print(action)
        # action = [[0,0,1.0,0], [0,0,0.5, 0]]  # FIXME: TEMPORARY
        for d, a in zip(self.drones, action): 
            d.update_act(a)
        try:
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
            print(f"overstepped {100 * i/i_overstep_period}% of the time")

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


