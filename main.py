from D435functions import start_D435, data_D435, exists_obstacle_ahead, stop_D435
from DRONEfunctions import start_connection, stop_connection, stop_mission, save_mission, change_altitude, move_forward, add_current_waypoint, upload_mission
from ALTIMETERfunctions import start_alt, stop_alt, getDistance_alt, exists_obstacle_under
import time
import serial
import dronekit_sitl

# Flight parameters
flightaltitude = 2 #m ???????(invent)??????? m *******************************************
speed = 5 #m/s ???????(invent)??????? m *******************************************

# Camera serial number (to identify it)
serialnumber_D435 = "829212070982" # ******************* NÚM. DE SÈRIE DE LA CÀMERA *******************

# Camera number of pixels
xmin = 0
xmax = 1280 
ymin = 0 
ymax = 720 

# Connection with altimeter
pipe_alt = None
while (pipe_alt == None):
    pipe_alt = start_alt()

# Connection with D435 camera
pipe_D435 = None
while pipe_D435 == None:
    pipe_D435 = start_D435(serialnumber_D435)

# Starts simulation
sitl = dronekit_sitl.start_default()

# Connection with drone
vehicle = None
while vehicle == None:
    vehicle = start_connection()
while not vehicle.is_armable:
    time.sleep(1)

#Upload mission from file
filename = 'mission.txt'
upload_mission(vehicle, filename)

try:
    while True:

        # Altitude data
        altitude = getDistance_alt(pipe_alt)

        # Check altitude
        while (altitude != flightaltitude):
            change_altitude(vehicle, flightaltitude - altitude)
    
        # Depth data
        depthframes = data_D435(pipe_D435)

        # Security parameters
        securitydistance = 2 # ???????(invent)??????? m *******************************************
        minpixels = 100 # ???????(invent)??????? pixels  *******************************************
    
        # Obstacle detected
        if (exists_obstacle_ahead(depthframes, securitydistance, xmin, ymin, xmax, ymax, minpixels) == True):
            # saves the initial mssion
            initialmission = save_mission(vehicle) 
            # stops the initial mission
            stop_mission(vehicle) 
            # obstacle avoidance loop
            Ah = 0.5 # ???????(invent)??????? m  *******************************************
            Ad = 0.5 # ???????(invent)??????? m  *******************************************
            d = 7 # ???????(invent)??????? m  *******************************************
            counter = 0
            while (exists_obstacle_ahead(depthframes, securitydistance, xmin, ymin, xmax, ymax, minpixels) == True):
                while (exists_obstacle_ahead(depthframes, securitydistance, xmin, ymin, xmax, ymax, minpixels) == True):
                    while (exists_obstacle_ahead(depthframes, d, xmin, ymin, xmax, ymax, minpixels) == True):
                        change_altitude(vehicle, Ah)
                        counter+=1
                    move_forward(vehicle, d - securitydistance, speed)
                obstacle_under = True
                while (exists_obstacle_ahead(depthframes, securitydistance + Ad, xmin, ymin, xmax, ymax, minpixels) == False and obstacle_under == True):
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