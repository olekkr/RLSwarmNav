
import numpy as np
from cflib.crazyflie.log import LogConfig
from mock_crazyflie import MockLogConfig
# from mock_crazyflie import MockLogConfig as LogConfig

from constants import *
from constants import BOUNDING_BOX



class ObsFactory:
    def __init__(self, mod_call_sig):
        self.mod_call_sig = mod_call_sig
        self.containers = []

    def generate(self):
        """construct and return the modules for the single drone"""
        modules = [globals()[n](**a) for (n, a) in self.mod_call_sig ]
        container = ObsContainer(modules)
        self.containers.append(container)
        container.inter_link(self.containers)
        return container
        

class ObsContainer: 
    def __init__(self, mods):
        """ construct container and link intra-drone obs """
        self.modules = mods

        for m in mods: 
            m.intra_link(self)
    
    def cf_init(self, scf):
        """ initializes container to be used in IRL swarm"""
        for m in self.modules:
            m.cf_init(scf)
        return self

    def get_data(self):
        for m in self.modules:
            m.manual_update_data()
        print([m.data for m in self.modules])
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
            raise Exception(f"query: {module_name} did not match a obsModule")

    def inter_link(self, containers):
        self.peer_containers = containers

    def inter_query(self, module_name):
        result = []
        mods_found = False
        for c in self.peer_containers:
            for m in c.modules:
                if m.name == module_name:
                    mods_found = True 
                    result.append(m.data) 
            if not mods_found: 
                raise Exception(f"query: {module_name} did not match a obsModule")
        # print(self.peer_containers, "AAAA")
        return np.stack(result)




class ObsMod:
    def __init__ (self, size):
        self.name = self.__class__.__name__
        self.size = size 
        self.data = np.zeros(size)
        self.log_conf = MockLogConfig("dummyconfig", 1)

    def intra_link(self, parent_container: ObsContainer):
        self.parent_container  = parent_container 
 
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

# class RelTargetPos(TargetPosObs):
#     def __init__(self, position):
#         super().__init__(position)
#         self.target_pos = self.data
#
#     def manual_update_data(self):
#         currpos = self.parent_container.intra_query("PosObs")
#         targetpos = self.parent_container.intra_query("TargetPosObs")
#         self.data = targetpos - currpos

# class RelDronePos(ObsMod): # going to do this later
#     def __init__(self, size):
#         super().__init__(size)
#
#     def manual_update_data(self):
#         self.size = 3 * len(self.parent_container.peer_containers)
#         positions = self.parent_container.inter_query("PosObs")
#         ego_pos = self.parent_container.intra_query("PosObs")
#         # print(positions, ego_pos, self.size)
#         self.data = positions - ego_pos



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
        self.data = np.zeros(size)

    def start (self):
        pass
    def stop(self):
        pass

