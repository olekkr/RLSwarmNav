
# from cflib.crazyflie.log import LogConfig
from mock_crazyflie import MockLogConfig as LogConfig
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
            log_conf.add_variable(f"{self.TOCName}.{k}", "float")
        scf.cf.log.add_config(log_conf)
        log_conf.data_received_cb.add_callback(self._cb)
        self.log_conf = log_conf
        return self

    def _cb(self, time, data, lg_conf):
        self.data = [data[f"{self.TOCName}.{k}"] for k in self.keys]

class PosObs(SimpleObs):
    def __init__(self):
        super().__init__("POS", "stateEstimate", ["x","y","z"])
class VelObs(SimpleObs):
    def __init__(self):
        super().__init__("VEL", "stateEstimate", ["vx","vy","vz"])
class RPYObs(SimpleObs):
    def __init__(self):
        super().__init__("RPY", "stateEstimate", ["roll","pitch","yaw"])
class AngRateObs(SimpleObs):
    def __init__(self):
        super().__init__("ANG_RATE", "stateEstimateZ", ["rateRoll","ratePitch","rateYaw"])
class QUATObs(SimpleObs):
    def __init__(self):
        super().__init__("ANG_RATE", "stateEstimate", ["qw","qx","qy", "qz"])

class ZeroObs(ObsMod):
    def __init__(self, size):
        super().__init__(f"{size} zeros", size)
        self.data = [0 for i in range(size)]

    def cf_init(self,scf): 
        self.log_conf = dummyLogConfig("dummy",1)
        return self


class dummyLogConfig ():
    def __init__(self,name, rate):
        pass
    def start(self):
        print("[DUMMY] START runned")
        pass
    def stop(self):
        pass

OBS_MODULES = [
    partial(PosObs), 
    partial(RPYObs),
    partial(VelObs),
    # partial(QUATObs),
    partial(AngRateObs),
    partial(ZeroObs, 45)]

