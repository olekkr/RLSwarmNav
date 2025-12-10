from constants import * 

if DEBUG:
    # from mock_crazyflie import MockSwarm as Swarm 
    # from mock_crazyflie import MockCommander as Commander 
    # from mock_crazyflie import MockMotionCommander as MotionCommander
    from mock_crazyflie import MockLogConfig as LogConfig
else:
    # from cflib.crazyflie.swarm import Swarm
    # from cflib.crazyflie.commander import Commander
    # from cflib.positioning.motion_commander import MotionCommander
    from cflib.crazyflie.log import LogConfig

class ObsMod:
    def __init__ (self, name, size):
        self.name = name
        self.size = size 
        self.data = [0 for i in range(size)]
        self.log_conf = dummyLogConfig("dummyconfig", 1)

    def __len__(self):
        return self.size 

    def __str__(self):
        return f"ObsMod:{self.name}"

    def start (self):
        self.log_conf.start()

    def stop(self):
        self.log_conf.stop()

    def cf_init(self, scf):
        pass 

    def get_obs(self):
        return self.data



class PosObs(ObsMod):
    def __init__(self):
        super().__init__("XYZ Pos", 3)

    def cf_init(self, scf):
        log_conf = LogConfig("stateEstimate", 1000/CTRL_FREQ)
        log_conf.add_variable("stateEstimate.x", "float")
        log_conf.add_variable("stateEstimate.y", "float")
        log_conf.add_variable("stateEstimate.z", "float")
        scf.cf.log.add_config(log_conf)
        log_conf.data_received_cb.add_callback(self._cb)
        self.log_conf = log_conf

    

    def _cb(self, time, data, lg_conf):
        self.data = [
                data["stateEstimate.x"],
                data["stateEstimate.y"],
                data["stateEstimate.z"],
        ]

class ZeroObs(ObsMod):
    def __init__(self, size):
        super().__init__(f"{size} zeros", size)
        self.obs = [0 for i in range(size)]

    def cf_init(self,scf): 
        self.log_conf = dummyLogConfig("dummy",1)

class dummyLogConfig ():
    def __init__(self,name, rate):
        pass
    def start(self):
        print("START runned")
        pass
    def stop(self):
        pass


    
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
