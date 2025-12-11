
from cflib.crazyflie.log import LogConfig
# from mock_crazyflie import MockLogConfig as LogConfig
from constants import *
from functools import partial


class ObsMod:
    def __init__ (self, name, size):
        self.name = name
        self.size = size 
        self.data = [0 for i in range(size)]
        self.log_conf = dummyLogConfig("dummyconfig", 1)

    def __len__(self):
        return self.size 

    def __str__(self):
        return f"(ObsMod:{self.name})"

    def start (self):
        self.log_conf.start()

    def stop(self):
        self.log_conf.stop()

    def cf_init(self, scf):
        return self



class SimpleObs(ObsMod):
    def __init__(self, name, TOCName, keys):
        super().__init__("Pos", len(keys))
        self.TOCName = TOCName
        self.keys = keys
        

    def cf_init(self, scf):
        log_conf = LogConfig(self.TOCName, 1000/CTRL_FREQ)
        for k in self.keys:
            log_conf.add_variable(f"{self.TOCName}.{k}", float)
        # log_conf.add_variable("stateEstimate.x", "float")
        # log_conf.add_variable("stateEstimate.y", "float")
        # log_conf.add_variable("stateEstimate.z", "float")
        scf.cf.log.add_config(log_conf)
        log_conf.data_received_cb.add_callback(self._cb)
        self.log_conf = log_conf
        return self

    def _cb(self, time, data, lg_conf):
        self.data = [data[f"{self.TOCName}.{k}"] for k in self.keys]
        # self.data = [
        #         data["stateEstimate.x"],
        #         data["stateEstimate.y"],
        #         data["stateEstimate.z"],
        # ]

class PosObs(SimpleObs):
    def __init__(self):
        super().__init__("Pos", "stateEstimate", ["x","y","z"])

class ZeroObs(ObsMod):
    def __init__(self, size):
        super().__init__(f"{size} zeros", size)
        self.data = [0 for i in range(size)]

    def cf_init(self,scf): 
        self.log_conf = dummyLogConfig("dummy",1)
        return self

        #need:  X        Y        Z       Q1   Q2   Q3   Q4   R       P       Y       VX       VY       VZ       WX       WY       WZ
        # state estimate logging


        # # Position observation module: 
        # lg_pos = LogConfig("stateEstimate", 1000/CTRL_FREQ)
        # lg_pos.add_variable("stateEstimate.x", "float")
        # lg_pos.add_variable("stateEstimate.y", "float")
        # lg_pos.add_variable("stateEstimate.z", "float")
        # scf.cf.log.add_config(lg_pos)
        # self.lg_state = lg_pos

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


    # ### in crazyflie
    # init in crazyflie 
    # extract from cf.log 
    
    # ### in training 
    # _observationspace size contribution
    # _compute observation 

    # ### common
    # shape 
    # __len__()
    # __str__

class dummyLogConfig ():
    def __init__(self,name, rate):
        pass
    def start(self):
        print("START runned")
        pass
    def stop(self):
        pass

OBS_MODULES = [
    partial(PosObs), 
    partial(ZeroObs, 54)]

