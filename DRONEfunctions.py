from dronekit import connect, VehicleMode, Command, LocationGlobal
from pymavlink import mavutil

# Starts connection with the drone (controller)
def start_connection():
    # connection options: SITL
    vehicle=connect('/dev/serial0',921600, wait_ready=True)
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
    # Save
    vehicle.commands.download()
    vehicle.commands.wait_ready()
    # Store
    missionvector=[]
    for waypoint in vehicle.commands:
        missionvector.append(waypoint)
    return missionvector

# Changes (+ increase, - decrease) the altitude Ah m
def change_altitude(vehicle, Ah):
    latitude = vehicle.location.lat
    longitude = vehicle.location.lon
    currentAlt = vehicle.location.alt
    newAlt = currentAlt + Ah
    newLocation = LocationGlobal(latitude, longitude, newAlt)
    vehicle.gotoGPS(newLocation)

# Moves along Ad m
def move_forward(vehicle, Ad, speed):
    At = Ad / speed
    vehicle.send_global_velocity(speed, 0, 0, At)

# Adds current location as waypoint in a mission
def add_current_waypoint(vehicle, missionvector):
    new_wp = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, vehicle.location.global_relative_frame.lat, vehicle.location.vehicle.location.global_relative_frame.alt.lon, )
    missionvector.append(new_wp)
    return missionvector
    
# Uploads a vector of waypoints as a mission
def upload_mission(vehicle, missionvector):
    for waypoint in missionvector:
        vehicle.commands.add(waypoint)
    vehicle.commands.upload()

# Uploads a mission from a file
def upload_mission_from_file(vehicle, aFileName):
    #Read mission from file
    cmds = vehicle.commands
    missionlist=[]
    with open(aFileName) as f:
        for i, line in enumerate(f):
            if i==0:
                if not line.startswith('QGC WPL 110'):
                    raise Exception('File is not supported WP version')
            else:
                linearray=line.split('\t')
                ln_index=int(linearray[0])
                ln_currentwp=int(linearray[1])
                ln_frame=int(linearray[2])
                ln_command=int(linearray[3])
                ln_param1=float(linearray[4])
                ln_param2=float(linearray[5])
                ln_param3=float(linearray[6])
                ln_param4=float(linearray[7])
                ln_param5=float(linearray[8])
                ln_param6=float(linearray[9])
                ln_param7=float(linearray[10])
                ln_autocontinue=int(linearray[11].strip())
                cmd = Command( 0, 0, 0, ln_frame, ln_command, ln_currentwp, ln_autocontinue, ln_param1, ln_param2, ln_param3, ln_param4, ln_param5, ln_param6, ln_param7)
                missionlist.append(cmd)
    #Clear existing mission from vehicle
    stop_mission(vehicle)
    #Add new mission to vehicle
    for command in missionlist:
        cmds.add(command)
    vehicle.commands.upload()