# Import functions
from D435functions import start_D435, exists_obstacle_ahead, stop_D435

# Camera parameters
serialnumber_D435 = "829212070982"
x_pixels = 640
y_pixels = 480

# Security parameters
securitydistance = 2 # m *******************************************
minpixels = 1000 # pixels *******************************************

# Connection with D435 camera
pipe_D435 = None
while pipe_D435 == None:
    pipe_D435 = start_D435(serialnumber_D435)

try:
    while True:
        if (exists_obstacle_ahead(pipe_D435, securitydistance, x_pixels, y_pixels, minpixels) == True):
            print("Obstacle detected")
        else:
            print("No obstacle")
            
finally:

    # Connection with camera
    stop_D435(pipe_D435)