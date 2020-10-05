import PyLidar3 as libALT
import time

# Starts connection with the altimeter
def start_alt(port_alt):
    obj = libALT.LIDARLite(port_alt)
    if obj.Connect():
        obj.StartScanning()
        return obj
    else:
        return None

# Returns altitude data
def data_alt(obj):
    altitude = obj.distance
    time.sleep(0.005)
    return altitude

# Establish (true, false) if there is an obstacle under the drone
def exists_obstacle_under(pipe_alt, flightaltitude, Ah):
    altitude_alt = data_alt(pipe_alt)
    if (altitude_alt == flightaltitude + Ah):
        return False
    else:
        return True

# Stops connection
def stop_alt(obj):
    obj.Disconnect()
    