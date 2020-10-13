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
        count = pipe_alt.in_waiting
        if (count > 8):
            recv = pipe_alt.read(9)
            pipe_alt.reset_input_buffer()
            if ((recv[0] == 'Y') and (recv[1] == 'Y')):
                low = int(recv[2].encode('hex'),16)
                high = int(recv[3].encode('hex'),16)
                distance = low + high * 256
                return distance

# Compares altitude and returns True if there is an obstacle below the drone or False if not
def exists_obstacle_under(pipe_alt, flightaltitude, change_altitude):
    if (getDistance_alt(pipe_alt) == flightaltitude + change_altitude):
        return False
    else:
        return True

# Stops connection with distance sensor
def stop_alt(pipe_alt):
    pipe_alt.close()