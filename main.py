# Import functions
from D435functions import start_D435, exists_obstacle_ahead, stop_D435
from DRONEfunctions import start_connection, stop_connection, stop_mission, save_mission, get_flight_altitude, change_altitude, move_forward, add_current_waypoint, test_mission, exists_obstacle_under, upload_mission

# Flight parameters
flightaltitude = 1 # m *******************************************
speed = 5 # m/s *******************************************

# Camera parameters
serialnumber_D435 = "829212070982"
x_pixels = 640
y_pixels = 480

# Connection with D435 camera
pipe_D435 = None
while pipe_D435 == None:
    pipe_D435 = start_D435(serialnumber_D435)

# Connection with drone
vehicle = None
while vehicle == None:
    vehicle = start_connection()

# Upload mission
test_mission(vehicle, flightaltitude, speed, 100)

try:
    while True:

        # Altitude data
        altitude = vehicle.rangefinder.distance

        # Check altitude (maximum 5% error)
        while (altitude >= 0.95 * flightaltitude and altitude <= 1.05 * flightaltitude):
            #get_flight_altitude(vehicle, flightaltitude)
            print("Current altitude: " + str(altitude) + " m | Flight altitude: " + str(flightaltitude) + " m")
            altitude = vehicle.rangefinder.distance

        # Security parameters
        securitydistance = 2 # m *******************************************
        minpixels = 1000 # pixels *******************************************

        # Obstacle detected
        if (exists_obstacle_ahead(pipe_D435, securitydistance, x_pixels, y_pixels, minpixels) == True):
            print("Obstacle detected")
            print("Starting avoidance mission")

            # Save initial mssion
            initialmission = save_mission(vehicle)
            print("Initial mission saved")

            # Stops the initial mission
            #stop_mission(vehicle)
            print("Initial mission stopped")

            # Obstacle avoidance parameters
            Ah = 0.5 # m ******************************************* IT MUST BE SMALLER THAN SECURITY DISTANCE
            Ad = 0.5 # m ******************************************* IT MUST BE SMALLER THAN SECURITY DISTANCE
            d = 3.5 # m ******************************************* IT MUST BE HIGHER THAN SECURITY DISTANCE

            # Obstacle avoidance loop
            counter = 0
            while (exists_obstacle_ahead(pipe_D435, securitydistance, x_pixels, y_pixels, minpixels) == True):
                print("[while 1] Obstacle at less than " + str(securitydistance) + " m")
                while (exists_obstacle_ahead(pipe_D435, securitydistance, x_pixels, y_pixels, minpixels) == True):
                    print("[while 2] Obstacle at less than " + str(securitydistance) + " m")
                    while (exists_obstacle_ahead(pipe_D435, d, x_pixels, y_pixels, minpixels) == True):
                        print("Obstacle at less than " + str(d) + " m")
                        #change_altitude(vehicle, Ah)
                        print("Increase altitude " + str(Ah) + " m")
                        counter += 1
                    #move_forward(vehicle, d - securitydistance, speed)
                    print("Move " + str(d - securitydistance) + " m")
                obstacle_under = True
                while (exists_obstacle_ahead(pipe_D435, securitydistance + Ad, x_pixels, y_pixels, minpixels) == False and obstacle_under == True):
                    if (exists_obstacle_under(vehicle, flightaltitude, counter*Ah) == False):
                        #get_flight_altitude(vehicle, flightaltitude)
                        print("Decrease altitude to fligh altitude")
                        obstacle_under = False
                    else:
                        print("Obstacle under the drone")
                        #move_forward(vehicle, Ad, speed)
                        print("Move " + str(Ad) + " m")

            # Return to initial mission
            print("Avoidance mission done. Going on with initial mission.")
            initialmission_edit = add_current_waypoint(vehicle, initialmission, flightaltitude)
            upload_mission(vehicle, initialmission_edit)

finally:

    # Connection with camera
    stop_D435(pipe_D435)

    # Connection with drone
    stop_connection(vehicle)