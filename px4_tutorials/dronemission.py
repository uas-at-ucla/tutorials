#!/bin/bash

# Import Dronekit-Python
from dronekit import connect, Command, LocationGlobal
from pymavlink import mavutil
from enum import Enum
import time, sys, argparse, math, subprocess, os

############################################################################
# Settings
############################################################################

# Connect to vehicle with this string
connection_string = '127.0.0.1:14540'
MAV_MODE_AUTO     = 4

# Parse connection argument and add preset flight paths
parser = argparse.ArgumentParser()
parser.add_argument("-c", "--connect", help="connection string")
args = parser.parse_args()
if args.connect:
    connection_string = args.connect

print ("""Please open QGroundControl and Gazebo if you have not already done so.
        You can open QGroundControl by navigating to the QGroundControl.AppImage file you downloaded
        and run ./QGroundControl.AppImage.
        You can open Gazebo by navigating to the Firmware repository you cloned and running
        make posix gazebo_plane.""")

###########################################################################
# Mission Manager Class
###########################################################################

# This class is the main class for creating missions and running them
class MissionManager:

    #  Class variables for command types
    class command_type(Enum):
        COMMAND_TAKEOFF = 0
        COMMAND_WAYPOINT = 1
        COMMAND_LAND = 2

    # Create vehicle
    vehicle = connect(connection_string, wait_ready=True)
    home_position_set = False

    # Create commands
    cmds = vehicle.commands
    
    # set home location
    home = vehicle.location.global_relative_frame

    # Constructor will initialize the program and run it
    def __init__(self):
        # Connect to the vehicle
        print ("Connecting")

        # Load commands
        self.cmds.clear()

    def PX4setMode(self, mavMode):
        self.vehicle._master.mav.command_long_send(self.vehicle._master.target_system, self.vehicle._master.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, mavMode, 0, 0, 0, 0, 0, 0)

    def get_location_offset_meters(self, original_location, dNorth, dEast, alt):
        """
        Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the
        specified `original_location`. The returned Location adds the entered `alt` value to the altitude of the `original_location`.
        The function is useful when you want to move the vehicle around specifying locations relative to
        the current vehicle position.
        The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
        For more information see:
        http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
        """
        earth_radius = 6378137.0 #Radius of "spherical" earth

        #Coordinate offsets in radians
        dLat = dNorth/earth_radius
        dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

        #New position in decimal degrees
        newlat = original_location.lat + (dLat * 180/math.pi)
        newlon = original_location.lon + (dLon * 180/math.pi)
        return LocationGlobal(newlat, newlon, original_location.alt + alt)

    def print_vehicle_state(self):
        # Display basic vehicle state
        print (" Type: %s" % self.vehicle._vehicle_type)
        print (" Armed: %s" % self.vehicle.armed)
        print (" System status: %s" % self.vehicle.system_status.state)
        print (" GPS: %s" % self.vehicle.gps_0)
        print (" Alt: %s" % self.vehicle.location.global_relative_frame.alt)

    def create_command(self, cmd_type, wp, vertical, horizontal, depth):
        if (cmd_type == self.command_type.COMMAND_TAKEOFF):
            print ("In takeoff")
            wp = self.get_location_offset_meters(self.home, vertical, horizontal, depth)
            cmd = Command(0, 0, 0, 
                          mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                          mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                          0, 1, 0, 0, 0, 0,
                          wp.lat, wp.lon, wp.alt)
        elif (cmd_type == self.command_type.COMMAND_WAYPOINT):
            wp = self.get_location_offset_meters(wp, vertical, horizontal, depth)
            cmd = Command(0, 0, 0,
                          mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                          mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                          0, 1, 0, 0, 0, 0,
                          wp.lat, wp.lon, wp.alt)
        elif (cmd_type == self.command_type.COMMAND_LAND):
            wp = self.get_location_offset_meters(self.home, vertical, horizontal, depth)
            cmd = Command(0, 0, 0,
                          mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                          mavutil.mavlink.MAV_CMD_NAV_LAND,
                          0, 1, 0, 0, 0, 0,
                          wp.lat, wp.lon, wp.alt)
        else:
            print ("Invalid command type")
            return

        self.cmds.add(cmd)
        return wp

    def run_mission(self):
        # wait for a home position lock
        while not self.home_position_set:
            print ("Waiting for home position...")
            time.sleep(1)

        # Print vehicle states
        self.print_vehicle_state()

        # Change to AUTO mode
        self.PX4setMode(MAV_MODE_AUTO)
        time.sleep(1)

        # Upload waypoints
        wp = ""
        wp = self.create_command(self.command_type.COMMAND_TAKEOFF, wp, 150, 150, 100)
        self.create_command(self.command_type.COMMAND_WAYPOINT, wp, 50, 0, 0)
        self.create_command(self.command_type.COMMAND_WAYPOINT, wp, 0, 50, 0)
        self.create_command(self.command_type.COMMAND_WAYPOINT, wp, -50, 0, 0)
        self.create_command(self.command_type.COMMAND_WAYPOINT, wp, 0, -850, 0)
        self.create_command(self.command_type.COMMAND_LAND, wp, 0, 0, 0)

        # Upload mission
        self.cmds.upload()
        time.sleep(2)

        # Arm vehicle
        self.vehicle.armed = True

        # Monitor mission execution
        nextwaypoint = self.vehicle.commands.next
        while (nextwaypoint < len(self.vehicle.commands)):
            if self.vehicle.commands.next > nextwaypoint:
                display_seq = self.vehicle.commands.next + 1
                print ("Moving to waypoint %s" % display_seq)
                nextwaypoint = self.vehicle.commands.next
            time.sleep(1)

        # Wait for the vehicle to land
        while (self.vehicle.commands.next > 0):
            time.sleep(1)

        # Disarm vehicle
        self.vehicle.armed = False
        time.sleep(1)

        # Close vehicle object before exiting script
        self.vehicle.close()
        time.sleep(1)

# Run mission
new_mission = MissionManager()

# Listeners
@MissionManager.vehicle.on_message('HOME_POSITION')
def listener(self, name, home_position):
    global home_position_set
    MissionManager.home_position_set = True

new_mission.run_mission()
