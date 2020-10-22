import serial
from dronekit import connect, VehicleMode, Command, LocationGlobal
from pymavlink import mavutil
from math import asin,cos,pi,sin

# Starts connection with the drone (controller)
def start_connection():
    vehicle=connect('/dev/serial0', baud=921600, wait_ready=True)
    return vehicle

# Stops connection with the drone
def stop_connection(vehicle):
    vehicle.close()

# Stops the initial mission 
def stop_mission(vehicle):
    vehicle.commands.clear()
    vehicle.commands.flush()

# Saves the actual mission & Returns it in a vector
def save_mission(vehicle):
    # Save mission
    vehicle.commands.download()
    vehicle.commands.wait_ready()
    # Store mission
    missionvector=[]
    for waypoint in vehicle.commands:
        missionvector.append(waypoint)
    return missionvector

# Changes to fligh altitude
def get_flight_altitude(vehicle, flightaltitude):
    # Current position
    latitude = vehicle.location.global_frame.lat
    longitude = vehicle.location.global_frame.lon
    # Final position
    newLocation = LocationGlobal(latitude, longitude, flightaltitude)
    # Move drone
    vehicle.gotoGPS(newLocation)

# Changes (+ increase, - decrease) the altitude Ah m
def change_altitude(vehicle, Ah):
    # Current position
    latitude = vehicle.location.global_frame.lat
    longitude = vehicle.location.global_frame.lon
    currentAlt = vehicle.location.alt
    # Final position
    newAlt = currentAlt + Ah
    newLocation = LocationGlobal(latitude, longitude, newAlt)
    # Move drone
    vehicle.gotoGPS(newLocation)

# Moves along Ad m
def move_forward(vehicle, Ad, speed):
    At = Ad / speed
    vehicle.send_global_velocity(speed, 0, 0, At)

# Adds current location as waypoint in a mission
def add_current_waypoint(vehicle, missionvector, flightaltitude):
    new_wp = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon, flightaltitude)
    missionvector.append(new_wp)
    return missionvector
    
# Uploads a vector of waypoints as a mission
def upload_mission(vehicle, missionvector):
    for waypoint in missionvector:
        vehicle.commands.add(waypoint)
    vehicle.commands.upload()

# Creates a mission (move along distance m) in order to test the code
def test_mission(vehicle, flightaltitude, speed, distance):
    # Current position
    lat = vehicle.location.global_frame.lat
    lon = vehicle.location.global_frame.lon
    heading = vehicle.heading
    # Final position
    finalPoint = pointRadialDistance(lat, lon, heading, distance/1000)
    # Create the mission
        # Take-Off:
    vehicle.commands.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    mavutil.mavlink.MAV_CMD_DO_SET_HOME, 0, 0, 1, 0, 0, 0, 0, 0, flightaltitude))
    vehicle.commands.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    mavutil.mavlink.MAV_CMD_DO_SET_HOME, 0, 0, 1, 0, 0, 0, 0, 0, flightaltitude))
    vehicle.commands.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 20, 0, 0, 0, 0, 0, flightaltitude))
    vehicle.commands.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0, 0, 0, speed, 0, 0, 0, 0, flightaltitude))
        # Mission
    vehicle.commands.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, finalPoint.lat, finalPoint.lon, flightaltitude))
        # Landing
    vehicle.commands.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    mavutil.mavlink.MAV_CMD_DO_LAND_START, 0, 0, 0, 0, 0, 0, finalPoint.lat, finalPoint.lon, flightaltitude))
    vehicle.commands.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, finalPoint.lat, finalPoint.lon, flightaltitude))
    # Upload the mission
    vehicle.commands.upload()
    # Arm the drone
    vehicle.mode = VehicleMode("AUTO")
    vehicle.armed = True

# Computes the final point given the current position (lat, lon) [º], bearing angle [º] and the distance [km] to move (straight)
def pointRadialDistance(lat1, lon1, bearing, distance):
    # Earth average radius
    rEarth = 6371.01 # km
    # Threshols for floating-point equality
    epsilon = 0.000001 
    # Conversions
    rlat1 = lat1 * pi/180
    rlon1 = lon1 * pi/180
    degreeBearing = ((360-bearing)%360)
    rbearing = degreeBearing * pi/180
    rdistance = (distance)  / rEarth 
    # Compute new latitude
    rlat = asin(sin(rlat1) * cos(rdistance) + cos(rlat1) * sin(rdistance) * cos(rbearing) )
    # Compute new longitude
    if cos(rlat) == 0 or abs(cos(rlat)) < epsilon: # Endpoint a pole
        rlon=rlon1
    else:
        rlon = ( (rlon1 - asin( sin(rbearing)* sin(rdistance) / cos(rlat) ) + pi ) % (2*pi) ) - pi
    # Conversions to degrees
    lat = rlat * 180/pi
    lon = rlon * 180/pi
    # New location (don't mind about altitude)
    return LocationGlobal(lat, lon, 0)

# Compares altitudes in order to know if there is an obstacle (True) or not (False) under the drone
def exists_obstacle_under(vehicle, flightaltitude, Ah):
    if (vehicle.rangefinder.distance < 0.95 * (flightaltitude + Ah)): # With a 5% of error
        return True
    else:
        return False