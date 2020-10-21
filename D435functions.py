import pyrealsense2 as libRS

# Starts connection with the camera
def start_D435(serialnumber):
    pipe_D435 = libRS.pipeline()
    cfg_D435 = libRS.config()
    cfg_D435.enable_device(serialnumber) 
    cfg_D435.enable_stream(libRS.stream.depth) # depth
    pipe_D435.start(cfg_D435) # start recording
    return pipe_D435

# Establish (True, False) if there is an obstacle ahead
def exists_obstacle_ahead(pipe_D435, securitydistance, x_pixels, y_pixels, minpixels):

    # Get depth data
    frames_D435 = pipe_D435.wait_for_frames() # frames
    depthframes = frames_D435.get_depth_frame() # filter --> only depth frames

    # Number of pixels at a distance less than the security one
    counter = 0 

    # Read depth data
    x = 0
    while x < x_pixels:
        y = 0
        while y < y_pixels:
            if(depthframes.get_distance(x,y) != 0 and depthframes.get_distance(x,y) < securitydistance): 
                counter+=1
            y+=1
        x+=1
    
    # Establish if there is (True) (False) or not an obstacle
    if counter > minpixels:
        return True
    else:
        return False
    
# Stop the connection with the camera
def stop_D435(pipe_D435):
    pipe_D435.stop()