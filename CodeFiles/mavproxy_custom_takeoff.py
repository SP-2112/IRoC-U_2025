import os
import time
from pymavlink import mavutil
from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings

class TakeoffLandModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(TakeoffLandModule, self).__init__(mpstate, "takeoff_land", "")
        self.target_altitude = 1  # Set desired altitude in meters
        self.hover_time = 5  # Hover duration in seconds
        self.has_taken_off = False
        self.has_landed = False
        self.add_command('takeoff_land', self.cmd_takeoff_land, "Takeoff and land module", [])

    def cmd_takeoff_land(self, args):
        """Command to start the takeoff and landing sequence"""
        self.takeoff()

    def takeoff(self):
        if self.has_taken_off:
            print("Already taken off!")
            return

        # Arm the vehicle
        print("Arming vehicle...")
        self.master.arducopter_arm()
        time.sleep(2)

        # Set mode to GUIDED
        print("Setting mode to GUIDED_NOGPS...")
        self.master.set_mode("GUIDED_NOGPS")
        time.sleep(2)

        # Send takeoff command
        print(f"Taking off to {self.target_altitude} meters...")
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, self.target_altitude
        )
        time.sleep(self.hover_time)  # Hover for some time

        self.has_taken_off = True
        self.land()

    def land(self):
        if self.has_landed:

            print("Already landed!")
            return

        # Send land command
        print("Landing...")
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0, 0, 0, 0, 0, 0, 0
        )
        self.has_landed = True

    def idle_task(self):
        pass  # No idle behavior needed

    def mavlink_packet(self, m):
        pass  # No MAVLink packet handling needed

def init(mpstate):
    return TakeoffLandModule(mpstate)