import numpy as np
from types import SimpleNamespace
import time

# -----------------------------
# Mock LOG CONFIG / CALLBACKS
# -----------------------------
class MockLogConfig:
    def __init__(self, name, period_in_ms):
        self.name = name
        self.period_in_ms = period_in_ms
        self.variables = []
        self.data_received_cb = SimpleNamespace(add_callback=self._add_callback)
        self._callback = None

    def add_variable(self, var, _type):
        self.variables.append(var)

    def _add_callback(self, cb):
        self._callback = cb

    def start(self):
        print(f"[MOCK] Started logging on {self.name}.")

    def stop(self):
        print(f"[MOCK] Stopped logging on {self.name}.")

    def simulate_tick(self):
        """Generate fake stateEstimate.x/y/z"""

        fake = {
            "stateEstimate.x": np.random.uniform(-1,1),
            "stateEstimate.y": np.random.uniform(-1,1),
            "stateEstimate.z": np.random.uniform(0,2),

            "stateEstimate.vx": np.random.uniform(-1,1),
            "stateEstimate.vy": np.random.uniform(-1,1),
            "stateEstimate.vz": np.random.uniform(-1,1),

            "stateEstimate.roll": np.random.uniform(-1,1),
            "stateEstimate.pitch": np.random.uniform(-1,1),
            "stateEstimate.yaw": np.random.uniform(-1,1),

            "stateEstimateZ.rateRoll": np.random.uniform(-1,1),
            "stateEstimateZ.ratePitch": np.random.uniform(-1,1),
            "stateEstimateZ.rateYaw": np.random.uniform(-1,1),

            "stateEstimate.qw": np.random.uniform(-1,1),
            "stateEstimate.qx": np.random.uniform(-1,1),
            "stateEstimate.qy": np.random.uniform(-1,1),
            "stateEstimate.qz": np.random.uniform(-1,1),


        }
        if self._callback:
            self._callback(time.time(), fake, MockLogConfig("[MOCK] logconfig", 20))


# -----------------------------
# Mock Commander
# -----------------------------
class MockCommander:
    def __init__(self, _): 
        pass 

    def send_position_setpoint(self, x, y, z, yaw):
        print(f"[MOCK] setpoint: x={x:.2f}, y={y:.2f}, z={z:.2f}, yaw={yaw:.2f}")

    def send_velocity_world_setpoint(self, vx, vy, vz, vYaw):
        print(f"[MOCK] VEL-setpoint: vx={vx:.2f}, vy={vy:.2f}, vvz={vz:.2f}, vYaw={vYaw:.2f}")

    def send_notify_setpoint_stop(self):
        print("[MOCK] stop setpoint stream")


# -----------------------------
# Mock MotionCommander
# -----------------------------
class MockMotionCommander:
    def __init__(self, cf):
        pass

    def take_off(self, x=0., y=0.):
        print("[MOCK] take_off")

    def land(self):
        print("[MOCK] landing")

    def __exit__(self, *args):
        print("[MOCK] motion commander exit")


# -----------------------------
# Mock CF (single drone)
# -----------------------------
class MockCF:
    def __init__(self):
        # match your code: scf.cf.platform.send_arming_request(True)
        self.platform = self
        self.log = SimpleNamespace(add_config=self._add_config)
        self._logs = []

    def send_arming_request(self, value):
        print(f"[MOCK] arming request: {value}")

    def _add_config(self, log_cfg):
        self._logs.append(log_cfg)

    def simulate_logs(self):
        for log in self._logs:
            log.simulate_tick()


# -----------------------------
# Mock SingleCachedCF (matches SCF object API)
# -----------------------------
class MockSingleCF:
    def __init__(self):
        self.cf = MockCF()


# -----------------------------
# Mock Swarm implementation
# -----------------------------
class MockSwarm:
    def __init__(self, URIs, factory=None):
        self._cfs = {uri: MockSingleCF() for uri in URIs}

    def parallel_safe(self, fn, args=None):
        """Run fn(scf, *args) for each drone."""
        if isinstance(args, dict):
            for uri, scf in self._cfs.items():
                extra = args.get(uri, [])
                fn(scf, *extra)
                scf.cf.simulate_logs()
        else:
            # no args → just apply to scf
            for scf in self._cfs.values():
                fn(scf)
                scf.cf.simulate_logs()

    def parallel(self, fn, args=None):
        return self.parallel_safe(fn, args)

    def open_links(self):
        pass 

    def close_links(self):
        pass 
    
    def reset_estimators(self):
        pass 

    def __exit__(self, *args):
        print("[MOCK] Swarm exit")

