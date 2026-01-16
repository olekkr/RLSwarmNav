import logging
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie, HighLevelCommander
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper


from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

from cflib.positioning.motion_commander import MotionCommander
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.crazyflie.commander import Commander



# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/100/2M/E7E7E7E706')

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
        scf.cf.platform.send_arming_request(True)
        simple_log_async(scf,lg_state)
        lg_state.start()

        mc = Commander(scf.cf)
        hlmc = MotionCommander(scf.cf)
        hlmc.take_off(0.2)
        time.sleep(2)

        mc.send_position_setpoint(0,0,0.5,0)
        time.sleep(3)
        print("landing")
        mc.send_velocity_world_setpoint(0,0,-0.3,0)
        time.sleep(3)
        mc.send_stop_setpoint()

        # hlmc.take_off(0.5,0.2) 
        # time.sleep(2)
        # hlmc.land()
        # time.sleep(3)
        hlmc.__exit__(None, None, None)
        mc.send_notify_setpoint_stop()

        # mc = Commander(scf.cf)
        # hlmc = MotionCommander(scf.cf)
        # mc.send_position_setpoint(1,0, 0.0, 0.5)

        # mc.send_full_state_setpoint([0,0,0.5], [0,0,0], [0,0,0], [0,0,0,1], 0, 0, 0)
        # time.sleep(3)
        # hlmc.land()
        #
        # time.sleep(2)
        #
        # mc.send_stop_setpoint()
        # hlmc.stop()

        lg_state.stop()

