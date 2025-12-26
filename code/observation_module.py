
import numpy as np
# from cflib.crazyflie.log import LogConfig
from mock_crazyflie import MockLogConfig
from mock_crazyflie import MockLogConfig as LogConfig
from constants import *
# from functools import partial

# TODO:refractor to use container
# TODO:need a hook to bind observations to each other on a single drone
# TODO:need a hook to bind observations to each other across drones
# TODO:need method to be called in _observationSpace to get the size of the observation space
# TODO:need method to be called in _computeObs 

class ObsFactory:
    def __init__(self, mod_call_sig):
        self.mod_call_sig = mod_call_sig
        self.containers = []

    def generate(self):
        """construct and return the modules for the single drone"""
        modules = [globals()[n](**a) for (n, a) in self.mod_call_sig ]
        container = ObsContainer(modules)
        self.containers.append(container)

        # HERE WE DO INTER-DRONE-link
        return container
        

class ObsContainer: 
    def __init__(self, mods):
        """ construct container and link intra-drone obs """
        self.modules = mods

        # HERE WE DO INTRA-DRONE-link
        for m in mods: 
            m.intra_link(self)
    
    def cf_init(self, scf):
        """ initializes container to be used in IRL swarm"""
        for o in self.modules:
            o.cf_init(scf)
        return self

    def get_data(self):
        # TODO: do manual update on mods here
        for m in self.modules:
            m.manual_update_data()
        return np.concat([m.data for m in self.modules], axis=0).flatten()

    def start(self):
        for m in self.modules:
            m.start()

    def stop(self):
        for m in self.modules:
            m.stop()

    def intra_query(self, module_name):
        mod_found = False 
        for m in self.modules:
            if m.name == module_name:
                mod_found = True 
                return m.data 
        if not mod_found: 
            raise Exception("query did not match a obsModule")


class ObsMod:
    def __init__ (self, size):
        self.name = self.__class__.__name__
        self.size = size 
        self.data = [0 for i in range(size)]
        self.log_conf = MockLogConfig("dummyconfig", 1)

    def intra_link(self, parent_container: ObsContainer):
        self.parent_container  = parent_container 
    # def inter_link(self, containers):
    #     pass
    def manual_update_data(self):
        pass

    def __len__(self):
        return self.size 

    def __str__(self):
        return f"(ObsMod:{self.name})"

    def start (self):
        self.log_conf.start()
        pass
    def stop(self):
        self.log_conf.stop()
        pass

    def cf_init(self, scf):
        return self


class TargetPosObs(ObsMod):
    def __init__(self, position):
        super().__init__(3)
        if position is None:
            self.data = BOUNDING_BOX.sample()
        else:
            raise Exception("not implemented")

    def start (self):
        pass
    def stop(self):
        pass

class RelTargetPos(TargetPosObs):
    def __init__(self, position):
        super().__init__(position)
        self.target_pos = self.data

    def manual_update_data(self):
        currpos = self.parent_container.intra_query("PosObs")
        targetpos = self.parent_container.intra_query("TargetPosObs")
        self.data = targetpos - currpos

class SimpleObs(ObsMod):
    def __init__(self,  TOCName, keys):
        super().__init__(len(keys))
        self.name = self.__class__.__name__
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
        self.data = np.array([data[f"{self.TOCName}.{k}"] for k in self.keys]).flatten()

class PosObs(SimpleObs):
    def __init__(self):
        super().__init__("stateEstimate", ["x","y","z"])
class VelObs(SimpleObs):
    def __init__(self):
        super().__init__("stateEstimate", ["vx","vy","vz"])
class RPYObs(SimpleObs):
    def __init__(self):
        super().__init__("stateEstimate", ["roll","pitch","yaw"])
class AngRateObs(SimpleObs):
    def __init__(self):
        super().__init__("stateEstimateZ", ["rateRoll","ratePitch","rateYaw"])
class QUATObs(SimpleObs):
    def __init__(self):
        super().__init__("stateEstimate", ["qw","qx","qy", "qz"])

class ZeroObs(ObsMod):
    def __init__(self, size):
        super().__init__(size)
        self.name = f"{size} zeros"
        self.data = [0 for i in range(size)]

    def start (self):
        pass
    def stop(self):
        pass

class dummyLogConfig ():
    def __init__(self,name, rate):
        pass
    def start(self):
        print("[DUMMY] START runned")
        pass
    def stop(self):
        pass

# OBS_MODULES = [
#     partial(PosObs),
#     partial(RPYObs),
#     partial(VelObs),
#     # partial(QUATObs),
#     partial(AngRateObs),
#     partial(TargetPosObs, None),
#     partial(ZeroObs, 42),
# ]
#
