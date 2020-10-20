import time
import serial
from dronekit import connect, VehicleMode, Command, LocationGlobal
import dronekit_sitl
from commonFunctions import *

from D435functions import start_D435, exists_obstacle_ahead, stop_D435
from DRONEfunctions import start_connection, stop_connection, stop_mission, save_mission, change_altitude, move_forward, add_current_waypoint, upload_mission
from ALTIMETERfunctions import start_alt, stop_alt, getDistance_alt, exists_obstacle_under



# Connection with drone
vehicle = None
while vehicle == None:
    vehicle = start_connection()
while not vehicle.is_armable:
    time.sleep(1)

print(vehicle.system_status.)


