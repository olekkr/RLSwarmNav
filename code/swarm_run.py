import time
import numpy as np 

from gym_pybullet_drones.utils.enums import ActionType

import cflib.crtp
from cflib.positioning.motion_commander import MotionCommander
from cflib.crazyflie.commander import Commander
from cflib.crazyflie.swarm import Swarm, CachedCfFactory
from cflib.crazyflie.log import LogConfig

from constants import *
import custom_env




URIs = [
    "radio://0/100/2M/E7E7E7E701",
    "radio://0/100/2M/E7E7E7E702",
    # "radio://0/100/2M/E7E7E7E703",
    # "radio://0/100/2M/E7E7E7E704",
    # "radio://0/100/2M/E7E7E7E705",
    # "radio://0/100/2M/E7E7E7E706",
    # "radio://0/100/2M/E7E7E7E708",
    # "radio://0/100/2M/E7E7E7E709",
    # "radio://0/100/2M/E7E7E7E710"
]



class Drone:
    
    def __init__(self, scf):
        self.lc = Commander(scf.cf)
        self.mc = MotionCommander(scf.cf)
        lg_state= LogConfig(name="stateEstimate", period_in_ms=1000/CTRL_FREQ)
        lg_state.add_variable("stateEstimate.x", "float")
        lg_state.add_variable("stateEstimate.y", "float")
        lg_state.add_variable("stateEstimate.z", "float")
        self.obs = np.zeros(57)
        # TODO: ADD MORE 

        scf.cf.log.add_config(lg_state)

        # TODO: sus code
        lg_state.data_received_cb.add_callback(self._update_state)

        self.lg_state = lg_state 

    def _update_state(self, _timestamp, data):
        # TODO: change 
        self.obs = data 
        print(data)

    def update_act(self, act):
        self.act = act
    
def _start(scf):
    scf.cf.platform.send_arming_request(True)
    

def _stop(_scf, drone:Drone):
    drone.lc.send_notify_setpoint_stop()
    drone.mc.land()
    drone.mc.__exit__(None,None,None)

def _act(_scf, drone:Drone):

    if ACTIONTYPE == ActionType.PID:
        drone.lc.send_position_setpoint(*drone.act)
    else: 
        print("not supported action type")
        exit()

    


class Runtime() : 
    def __init__(self, URIs=URIs):
        self.swarm = Swarm(URIs,CachedCfFactory(rw_cache='./cache'))
        print("connected to: ", self.swarm._cfs)
        self.drones = [Drone(cf) for cf in self.swarm._cfs.values() ]
        self.droneArg ={uri: [drone] for uri, drone in zip(self.drones, URIs)} 
        print(f"test: {self.drones}")
        self.policy = custom_env.load_policy()
        self.started = False
        self.obs = np.empty_like(self.policy.observation_space.shape)
        
    def _start_drones(self):
        # self.swarm.reset_estimators() # FIXME
        try:
            self.swarm.parallel_safe(_start)
            self.started = True
        except:
            self.swarm.parallel(_stop)
        
    def _stop_drones(self):
        self.swarm.parallel(_stop, self.drones)
        self.started = False

    def _collect_obs(self): 
        self.obs = np.stack([np.array(d.obs) for d in self.drones],axis=0)
        
    def _step(self):
        self._collect_obs()
        action = self.policy.predict(self.obs)
        for d, a in zip(self.drones, action): 
            d.update_act(a)
        try:
            self.swarm.parallel_safe(_act, self.droneArg)
        except Exception as e:
            print("failed with exception:", e, "\nstopping.")
            self.swarm.parallel(_stop)


    def run(self, duration):
        if not self.started:
            self._start_drones()

        start = time.perf_counter()

        target_period = 1/CTRL_FREQ 
        while True: 
            last_tick = time.perf_counter()

            self._step()
            now = time.perf_counter()
            if now - start > duration:
                break
            time.sleep(last_tick-now+target_period)



    def __exit__(self):
        self.swarm.__exit__(None,None,None)




if __name__ == "__main__":
    cflib.crtp.init_drivers()

    drone = Runtime() 
    drone.run(-1)


