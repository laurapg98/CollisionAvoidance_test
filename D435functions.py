import pyrealsense2 as libRS

# Starts connection with the camera
def start_D435(serialnumber):
    pipe_D435 = libRS.pipeline()
    cfg_D435 = libRS.config()
    cfg_D435.enable_device(serialnumber) 
    cfg_D435.enable_stream(libRS.stream.depth) # depth
    pipe_D435.start(cfg_D435) # start recording
    return pipe_D435
    
# Returns depth data
def data_D435(pipe_D435):
    frames_D435 = pipe_D435.wait_for_frames() # frames
    depthframes = frames_D435.get_depth_frames() # filter --> only depth frames
    return depthframes

# Establish (true, false) if there is an obstacle ahead
def exists_obstacle_ahead(depthframes, securitydistance, xmin, ymin, xmax, ymax, minpixels):
    counter=0 # number of pixels at a distance less than the security one
    # frames matrix search
    x=xmin
    while(x<=xmax):
        y=ymin
        while(y<=ymax):
            if(depthframes.get_distance(x,y)!=0 and depthframes.get_distance(x,y)<securitydistance): 
                counter+=1
            y+=1
        x+=1
    # establish if there is or not an obstacle
    if (counter>minpixels):
        return True
    else:
        return False
    
# Stop the connection with the camera
def stop_D435(pipe_D435):
    pipe_D435.stop()