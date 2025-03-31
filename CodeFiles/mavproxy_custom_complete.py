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
        my_lat,my_lon,_ = self.get_gps_data()
        self.set_home_position(my_lat,my_lon) 

        self.arm_and_takeoff(5)

        self.set_mode("ALT_HOLD") # set mode to alt hold

        time.sleep(5) # wait for 5 seconds

        self.set_mode("LAND") # set mode to land

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

    def get_global_alt(self):

    #Altitude of the drone through the barometer.

    # Request the GLOBAL_POSITION_INT message
        self.master.mav.command_long_send(
            self.master.target_system,                      # Target system
            self.master.target_component,                   # Target component
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,   # MAVLink command to request a specific message
            0,                                         # Confirmation
            33,                                        # Message ID of GLOBAL_POSITION_INT
            0, 0, 0, 0, 0, 0                           # Unused parameters
        )

        # Wait for the GLOBAL_POSITION_INT message response
        msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)

        relative_alt = msg.relative_alt / 1000.0  # Convert to meters (above ground level)
    
        # Return the extracted data
        return relative_alt


 

    def  get_gps_data(self):
            """
            Make fake GPS data for the drone.
            """

            lat,lon,alt = 15.485296390754874, 74.93648325450908, 0 #IIT Dharwad location
        
            
            return lat , lon , alt 

    
    def get_drone_location(self):
        """Function to get GPS location"""
        lat,lon,alt = self.get_gps_data()
        return lat, lon



    def reset_global_origin(self,latitude, longitude, altitude):
        """
        Reset the global origin for the autopilot's local position calculations.
        """
        self.master.mav.set_gps_global_origin_send(
            self.master.target_system,            # Target system ID
            int(latitude * 1e7),             # Latitude in degrees * 1e7
            int(longitude * 1e7),            # Longitude in degrees * 1e7
            int(altitude * 1000)             # Altitude in millimeters
        )
        print(f"Global origin reset to: Lat={latitude}, Lon={longitude}, Alt={altitude}m")


    def send_vp_to_nav(self,x_global, y_global, alt):
        """
        Sends a vision position estimate message to the autopilot using MAVLink.
        """

        # Send VISION_POSITION_ESTIMATE message
        self.master.mav.global_vision_position_estimate_send(
            0,  # Timestamp (in microseconds)
            x_global,         # X position (latitude in meters or local frame)
            y_global,         # Y position (longitude in meters or local frame)
            alt,              # Z position (altitude in meters)
            0,                # Roll angle (radians)
            0,                # Pitch angle (radians)
            0,                # Yaw angle (radians)
            [10, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1, 0,1], # covariance matrix for vision estimates. First two non zeros are the uncertainty in x and y respectively
        )

        # Debug information
        print(f"Sent VISION_POSITION_ESTIMATE: Lat={x_global}, Lon={y_global}, Alt={alt}")


    def takeoff_drone(self,target_alt):
        """
        Commands the drone to take off to a specified altitude.
        """
        print(f"Initiating takeoff to {target_alt} meters...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0, 0, 0,
            target_alt
        )
        print("Takeoff command sent.")
        
    def force_arm_message_send(self):
        """
        Forcefully arms the drone, bypassing safety checks.
        """
        print("Forcefully arming the drone...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,                  # Confirmation
            1,                  # Arm (1 to arm, 0 to disarm)
            21196,              #  Force arm bypass code (magic number)
            0, 0, 0, 0, 0       # Unused parameters
        )
    
    def is_drone_armed(self):
        """
        Checks the armed state by monitoring SYS_STATUS message.
        """
        return True if self.master.motors_armed() else False




    def set_initial_position(self):
        """Set initial position to the drone."""
        for _ in range(10): # drone doesnt listen if told once
            
            # send_to_mav(22.6121311, 71.1460848, 200.0,1,0,0,0  )
            lat_gps,long_gps,alt_gps = self.get_gps_data()
            self.reset_global_origin(lat_gps, long_gps, alt_gps)    
            orig_lat,orig_lon,origin_alt = lat_gps,long_gps,alt_gps
            print(lat_gps,long_gps,alt_gps )
            print(time.time())
            time.sleep(2)

        print("Initial position origin set...")
        for i in range(15):
            # send initial location to the drone for some time. 
            # (This makes the drone happy and ready to arm)
            
            self.send_vp_to_nav(0,0,0) # initial position for arming
            time.sleep(0.2)
        print("Drone now can be force armed...")
        return  orig_lat,orig_lon,origin_alt 


    def set_mode(self,mode):
        """
        Sets the flight mode of the drone.
        :param mode: The desired mode as a string (e.g., "AUTO", "GUIDED", "STABILIZE").
        """
        # Get the mode ID from the mode string
        mode_id = self.master.mode_mapping().get(mode)
        
        if mode_id is None:
            print(f"Unknown mode: {mode}")
            return False

        # Send the command to set the mode
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )

        # Wait for the mode to be confirmed
        ack = self.master.recv_match(type='HEARTBEAT', blocking=True)
        if ack.custom_mode == mode_id:
            print(f"Mode successfully set to {mode}.")
            return True
        time.sleep(0.1)

            
        print(f"Mode set to {mode} FAILED.")
        return False
    

    def arm_and_takeoff(self,target_alt):
        """Arm and take off drone and take to height. Function will release 
    control only when drone reaches target height, else it will keep on 
    retrying and rearming"""
    
        self.send_vp_to_nav(0,0,0) # send 0,0,0 after few lines of code for matching frequency of 5hz
        self.set_mode("GUIDED")
        
        self.force_arm_message_send()
        time.sleep(0.2)
        
        print("Arming...")
        self.takeoff_drone(target_alt)
        print("Trying to take off...")
        counter = 0
        while True: 
            self.send_vp_to_nav(0,0,0)
            alt = self.get_global_alt()
            print("Current altitude = ", alt)
            print(self.is_drone_armed())
            if abs(alt  - target_alt) <1: 
                self.set_initial_position() 
                    # set initial position to remove any drifts that would have been 
                    # caused  while drone got armed
                return True
            time.sleep(0.2)
            if not self.is_drone_armed(): # sometimes drone disarms due to error
                self.set_initial_position()
                print("Some error in arming.. Retrying...")
                return self.arm_and_takeoff(target_alt)
        

    def set_home_position(self,lat,lon):
        self.master.mav.command_long_send(
                self.master.target_system, 
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_HOME,
                1,                      # set position
                0,                      # param1
                0,                      # param2
                0,                      # param3
                0,                      # param4
                lat , # Current Lat
                lon,  # CurrentLong
                0   # alt
                ) 
    def set_viso_type(self):
        self.master.mav.command_long_send

def init(mpstate):
    return TakeoffLandModule(mpstate)