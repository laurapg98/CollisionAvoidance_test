import serial

# Starts connection with distance sensor TF mini
def start_alt():
    pipe_alt = serial.Serial("/dev/ttyAMA0", 115200)
    while (pipe_alt.is_open == False):
        pipe_alt.open()
    return pipe_alt

# Returns distance measure 
def getDistance_alt(pipe_alt):
    while True:
        bytes_serial = pipe_alt.in_waiting
        if (count > 8):
            recv = pipe_alt.read(9)
            pipe_alt.reset_input_buffer()
            if bytes_serial[0] == 0x59 and bytes_serial[1] == 0x59: # for Python 3   
                distance = bytes_serial[2] + bytes_serial[3]*256 
                return(distance)
                ser.reset_input_buffer()

# Compares altitude and returns True if there is an obstacle below the drone or False if not
def exists_obstacle_under(pipe_alt, flightaltitude, change_altitude):
    if (getDistance_alt(pipe_alt) == flightaltitude + change_altitude):
        return False
    else:
        return True

# Stops connection with distance sensor
def stop_alt(pipe_alt):
    pipe_alt.close()