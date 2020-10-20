import time
import serial
from dronekit import connect, VehicleMode, Command, LocationGlobal
import dronekit_sitl
from commonFunctions import *

# Import functions
from D435functions import start_D435, exists_obstacle_ahead, stop_D435
from DRONEfunctions import start_connection, stop_connection, stop_mission, save_mission, change_altitude, move_forward, add_current_waypoint, upload_mission
from ALTIMETERfunctions import start_alt, stop_alt, getDistance_alt, exists_obstacle_under

# Flight parameters
# m ???????(invent)??????? m *******************************************
flightaltitude = 2
# m/s ???????(invent)??????? m *******************************************
speed = 5

# Camera serial number (to identify it)
serialnumber_D435 = "829212070982"

# Camera number of pixels
xmin = 0
xmax = 640
ymin = 0
ymax = 480

# Connection with altimeter
pipe_alt = None
while (pipe_alt == None):
    pipe_alt = start_alt()

# Connection with D435 camera
pipe_D435 = None
while pipe_D435 == None:
    pipe_D435 = start_D435(serialnumber_D435)


# Connection with drone
vehicle = None
while vehicle == None:
    vehicle = start_connection()
while not vehicle.is_armable:
    time.sleep(1)

# Upload mission from file
# upload_mission(vehicle, filename)
global cmds

cmds = vehicle.commands

lat = vehicle.location.global_frame.lat
lon = vehicle.location.global_frame.lon
heading = vehicle.heading

finalPoint = pointRadialDistance(lat, lon, heading, 0.1)

# TAKEOFF
cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                 mavutil.mavlink.MAV_CMD_DO_SET_HOME, 0, 0, 1, 0, 0, 0, 0, 0, flightaltitude))
cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                 mavutil.mavlink.MAV_CMD_DO_SET_HOME, 0, 0, 1, 0, 0, 0, 0, 0, flightaltitude))
cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                 mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 20, 0, 0, 0, 0, 0, flightaltitude))
cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                 mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0, 0, 0, speed, 0, 0, 0, 0, flightaltitude))
# MISSION

cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                 mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, finalPoint.lat, finalPoint.lon, flightaltitude))

# LANDING

cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                 mavutil.mavlink.MAV_CMD_DO_LAND_START, 0, 0, 0, 0, 0, 0, finalPoint.lat, finalPoint.lon, flightaltitude))

cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                 mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, finalPoint.lat, finalPoint.lon, flightaltitude))

cmds.upload()

vehicle.mode = VehicleMode("AUTO")
vehicle.armed = True


try:
    while True:

        # Altitude data
        # altitude = getDistance_alt(pipe_alt)
        altitude = vehicle.sonarrange

        # Check altitude
        while (altitude != flightaltitude):
            change_altitude(vehicle, flightaltitude - altitude)

        # Security parameters
        # ???????(invent)??????? m *******************************************
        securitydistance = 2
        # ???????(invent)??????? pixels  *******************************************
        minpixels = 100

        # Obstacle detected
        if (exists_obstacle_ahead(pipe_D435, securitydistance, xmin, ymin, xmax, ymax, minpixels) == True):
            # saves the initial mssion
            initialmission = save_mission(vehicle)
            # stops the initial mission
            stop_mission(vehicle)
            # obstacle avoidance loop
            # ???????(invent)??????? m  *******************************************
            Ah = 0.5
            # ???????(invent)??????? m  *******************************************
            Ad = 0.5
            # ???????(invent)??????? m  *******************************************
            d = 7
            counter = 0
            while (exists_obstacle_ahead(pipe_D435, securitydistance, xmin, ymin, xmax, ymax, minpixels) == True):
                while (exists_obstacle_ahead(pipe_D435, securitydistance, xmin, ymin, xmax, ymax, minpixels) == True):
                    while (exists_obstacle_ahead(pipe_D435, d, xmin, ymin, xmax, ymax, minpixels) == True):
                        change_altitude(vehicle, Ah)
                        counter += 1
                    move_forward(vehicle, d - securitydistance, speed)
                obstacle_under = True
                while (exists_obstacle_ahead(pipe_D435, securitydistance + Ad, xmin, ymin, xmax, ymax, minpixels) == False and obstacle_under == True):
                    if (exists_obstacle_under(pipe_alt, flightaltitude, counter*Ah) == False):
                        change_altitude(vehicle, -counter*Ah)
                        obstacle_under = False
                    else:
                        move_forward(vehicle, Ad, speed)
            # add current point to initial mission
            initialmission_edit = add_current_waypoint(vehicle, initialmission)
            upload_mission(vehicle, initialmission_edit)

finally:

    # Connection with altimeter
    stop_alt(pipe_alt)

    # Connection with camera
    stop_D435(pipe_D435)

    # Connection with drone
    stop_connection(vehicle)

    # Stops simulation
    if sitl is not None:
        sitl.stop()
