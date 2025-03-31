from pymavlink import mavutil
from MAVProxy.modules.lib import mp_module
import time

class SetGPSModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super().__init__(mpstate, "setgps", "Set GPS location and home position")
        self.add_command('setgps', self.cmd_setgps, "Set the GPS location as specified in the program")
        self.master = self.get_master()
        self.lat, self.lon, self.alt = 15.485296390754874, 74.93648325450908, 0  # IIT Dharwad
    
    def get_master(self):
        return mavutil.mavlink_connection('tcp:127.0.0.1:14550')

    def reset_global_origin(self):
        """Reset the global origin for the autopilot's local position calculations."""
        self.master.mav.set_gps_global_origin_send(
            self.master.target_system,
            int(self.lat * 1e7),
            int(self.lon * 1e7),
            int(self.alt * 1000)
        )
        self.console.writeln(f"Global origin set to: Lat={self.lat}, Lon={self.lon}, Alt={self.alt}m")
    
    def set_home_position(self):
        """Set the home position."""
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_HOME,
            1, 0, 0, 0, 0, self.lat, self.lon, self.alt
        )
        self.console.writeln(f"Home position set to: Lat={self.lat}, Lon={self.lon}")
    
    def send_vision_position(self):
        """Send vision position estimate to the autopilot."""
        for _ in range(10):  # Ensure the drone receives the message multiple times
            self.master.mav.global_vision_position_estimate_send(
                0, self.lat, self.lon, self.alt, 0, 0, 0,
                [10, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1, 0, 1]
            )
            time.sleep(0.2)
        self.console.writeln("Vision position estimates sent.")
    
    def cmd_setgps(self, args):
        """Command to set the GPS location and home position."""
        self.console.writeln("Setting GPS location and home position...")
        self.reset_global_origin()
        self.set_home_position()
        self.send_vision_position()
        self.console.writeln("GPS setup complete.")

def init(mpstate):
    return SetGPSModule(mpstate)
