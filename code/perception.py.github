import numpy as np
import cv2
import time

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(162, 162, 162)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))
                            
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated

def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result  
    return xpix_translated, ypix_translated


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):         
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    mask = cv2.warpPerspective(np.ones_like(img[:,:,0]), M, (img.shape[1], img.shape[0]))# create array of one the same size as warped
   
    return warped, mask
   
def rover_stuck(Rover):
    if Rover.vel < .1 and Rover.throttle > .1:
        Rover.stuck_time = Rover.stuck_time + 1
        print(Rover.stuck_time)
    if  Rover.vel < .1 and Rover.stuck_time > 200:
        Rover.reverse = 'True'
        Rover.stuck_time = 0
    if len(Rover.nav_angles) <= Rover.stop_forward and Rover.vel == 0 and Rover.reverse == 'False':
        #Rover at a dead end need to do 180
        Rover.turn180 = 'True'
        Rover.reverse = 'False'
    return Rover

# this cell is the function for finding moon rocks it takes 
def find_rocks(img, levels=(90,90,40)):
    rockpix = ((img[:,:,0]>levels[0]) \
              & (img[:,:,1]>levels[1]) \
              & (img[:,:,2]<levels[2]))
    
    color_select = np.zeros_like(img[:,:,0])
    color_select[rockpix] = 1
    
    return color_select

    
# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
# Define a function to pass stored images to
# reading rover position and yaw angle from csv file
# This function will be used by moviepy to create an output video

    # TODO: 
    # 1) Define source and destination points for perspective transform & destination points
    
# Define calibration box in source (actual) and destination (desired) coordinates
# These source and destination points are defined to warp the image
# to a grid where each 10x10 pixel square represents 1 square meter
# The destination box will be 2*dst_size on each side
    dst_size = 5 
# Set a bottom offset to account for the fact that the bottom of the image 
# is not the position of the rover but a bit in front of it
# this is just a rough guess, feel free to change it!
    bottom_offset = 6
    image = Rover.img
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[image.shape[1]/2 - dst_size, image.shape[0] - bottom_offset],
                  [image.shape[1]/2 + dst_size, image.shape[0] - bottom_offset],
                  [image.shape[1]/2 + dst_size, image.shape[0] - 2*dst_size - bottom_offset], 
                  [image.shape[1]/2 - dst_size, image.shape[0] - 2*dst_size - bottom_offset],
                  ])
    warped, mask = perspect_transform(Rover.img, source, destination)
    
    # 2) Apply perspective transform
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    # 4) update Rover.vions_image the image on the left side of the screen
    threshed = color_thresh(warped)
    Rover.vision_image[:,:,2] = threshed * 255      #multiply the binary threshed image by 255 converting to blu
    
    #take the transformed binary map and converto to float32 sub 1 from the map array and then take the absolute value(no negatives) multiply by the mask to cancel the portion of the array outside the field of the camera.
    obs_map = np.absolute(np.float32(threshed)-1) * mask
    Rover.vision_image[:,:,2] = threshed * 255      #multiply the binary threshed image by 255 converting to blue
    Rover.vision_image[:,:,0] = obs_map * 255       #multiply the binary threshed image by 255 converting to red    
    xpix, ypix = rover_coords(threshed)
    
    # convert binary image to rover coords and save to variable xpix, ypix
    # 5) Convert rover-centric pixel values to world coords
    # 6) Update worldmap (to be displayed on right side of screen)
    # xpos = data object which is a read in value from a .csv file.  data.count is the index number
    xpos = Rover.pos[0] 
    ypos = Rover.pos[1]
    yaw = Rover.yaw
    # ypos = data object which is a read in value from a .csv file.  data.count is the index number
    # yaw = data object which is a read in value from a .csv file.  data.count is the index number
    world_size = Rover.worldmap.shape[0]
    scale = dst_size * 2
    
    #add an adjustment to pix_to_world as a function of dst_size
    #call pix_to_world module and input the arguments
    x_world, y_world = pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale)
    Rover.rover_x_world, Rover.rover_y_world = x_world, y_world
    
    #call rover_coords module to convert obs_map to rover coordinates
    obsxpix, obsypix = rover_coords(obs_map)
    
    #convert obs rover coordinates to world coordinates
    obs_x_world, obs_y_world = pix_to_world(obsxpix, obsypix, xpos, ypos, yaw, world_size, scale)
    Rover.worldmap[y_world, x_world, 2] += 10
    Rover.worldmap[obs_y_world, obs_x_world, 0] += 10
    dist, angles = to_polar_coords(xpix, ypix)
    Rover.nav_angles = angles
    
    #find out if the rover is stuck or at a deadend
    if Rover.vel < .1 and Rover.throttle > .1:
        Rover.stuck_time = Rover.stuck_time + 1
        print(Rover.stuck_time)
    if  Rover.vel < .1 and Rover.stuck_time > 200:
        Rover.reverse = 'True'
        Rover.stuck_time = 0
    if len(Rover.nav_angles) <= Rover.stop_forward and Rover.vel == 0:
        #Rover at a dead end need to do 180
        Rover.turn180 = 'True'
        Rover.reverse = 'False'
            
    #call function to look for rocks and assign to rock_map
    rock_map = find_rocks(warped, levels=(90,90,40))
    #if rock_map returns any results then run the code below
    if rock_map.any():
        rock_x, rock_y = rover_coords(rock_map)
        rock_x_world, rock_y_world = pix_to_world(rock_x, rock_y, Rover.pos[0], Rover.pos[1], Rover.yaw, world_size, scale)
        rock_dist, rock_ang = to_polar_coords(rock_x, rock_y)
        Rover.rock_x_world = rock_x_world
        Rover.rock_y_world = rock_y_world
        Rover.rock_dist = rock_dist
        Rover.rock_ang = rock_ang
        rock_idx = np.argmin(rock_dist)
        rock_xcen = rock_x_world[rock_idx]
        rock_ycen = rock_y_world[rock_idx]
        Rover.worldmap[rock_ycen, rock_xcen, 1] = 255
        Rover.vision_image[:,:,1] = rock_map * 255
    else:
        Rover.vision_image[:,:,1] = 0
        
        
    return Rover