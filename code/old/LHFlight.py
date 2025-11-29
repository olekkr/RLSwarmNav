import logging
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper


from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

from cflib.positioning.motion_commander import MotionCommander
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.crazyflie.commander import Commander



# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/100/2M/E7E7E7E703')

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

def log_callback(timestamp, data, logconf):
    # print('[%d][%s]: %s' % (timestamp, logconf.name, data))
    print(f"({timestamp}, {logconf.name}, {data}")

def simple_log_async(scf, logconf):
    cf = scf.cf
    cf.log.add_config(logconf)
    logconf.data_received_cb.add_callback(log_callback)




if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    lg_state= LogConfig(name="stateEstimate", period_in_ms=100)
    lg_state.add_variable("stateEstimate.x", "float")
    lg_state.add_variable("stateEstimate.y", "float")
    lg_state.add_variable("stateEstimate.z", "float")


    # group = "stateEstimate"
    # name = "estimator"

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:

        # simple_connect()
        # simple_log(scf, lg_stab)
        # simple_log_async(scf,lg_stab)
        scf.cf.platform.send_arming_request(True)
        simple_log_async(scf,lg_state)
        lg_state.start()
        
        mc = Commander(scf)
        # mc.send_position_setpoint(-1,0, 0.5, 0)

        mc.send_full_state_setpoint([0,0,1], [0,0,0], [0,0,0], [0,0,0,1], 0, 0, 0)
        time.sleep(3)

        mc.send_stop_setpoint()


        lg_state.stop()

