import numpy as np
import cv2

def color_thresh(img):
    frame = img
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    hue,saturation,brightness = cv2.split(hsv)
    ret, brightness_thresh = cv2.threshold(brightness, 155, 255, cv2.THRESH_BINARY);
    
    return brightness_thresh

def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image. 
    x_pixel = np.absolute(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1] / 2).astype(np.float)
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

# Define a function to apply a rotation to pixel positions
def rotate_pix(xpix, ypix, yaw):
    # TODO:
    # Convert yaw to radians
    # Apply a rotation
    yaw_rad = yaw * np.pi / 180 
    xpix_rotated = np.cos(yaw_rad) * xpix - np.sin(yaw_rad) * ypix
    ypix_rotated = np.cos(yaw_rad) * xpix + np.sin(yaw_rad) * ypix
    # Return the result  
    return xpix_rotated, ypix_rotated

# Define a function to perform a translation
def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # TODO:
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot/scale + xpos)
    ypix_translated = (ypix_rot/scale + ypos)
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


def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    mask = cv2.warpPerspective(np.ones_like(img[:,:,0]), M, (img.shape[1], img.shape[0]))# create array of one the same size as warped
    
    
    return warped, mask

def find_rocks(img, levels=(100,100,60)):
    rockpix = ((img[:,:,0]>levels[0]) \
              & (img[:,:,1]>levels[1]) \
              & (img[:,:,2]<levels[2]))
    
    color_select = np.zeros_like(img[:,:,0])
    #print(color_select[rockpix])
    color_select[rockpix] = 1
    
    return color_select

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

def process_image(data):
    img = data.img

    dst_size = 5 
    bottom_offset = 6

    # 1) Define source and destination points for perspective transform
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[img.shape[1]/2 - dst_size, img.shape[0] - bottom_offset],
                      [img.shape[1]/2 + dst_size, img.shape[0] - bottom_offset],
                      [img.shape[1]/2 + dst_size, img.shape[0] - 2*dst_size - bottom_offset], 
                      [img.shape[1]/2 - dst_size, img.shape[0] - 2*dst_size - bottom_offset],
                      ])

    scale = dst_size*2
    xpos = data.pos[0]
    ypos = data.pos[1]
    yaw = data.yaw
    world_size = data.worldmap.shape[0]
    output_image = np.zeros((img.shape[0] + data.worldmap.shape[0], img.shape[1]*2, 3))
    output_image[0:img.shape[0], 0:img.shape[1]] = img

    # 2) Apply perspective transform
    warped, mask = perspect_transform(img, source, destination)

    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    threshed = color_thresh(warped)
    obs_map = np.absolute(np.float32(threshed)-255)*mask
    
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
    # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
    #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
    #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image

    # data.vision_image[:,:,2] = np.ones((data.vision_image.shape[0], data.vision_image.shape[1])) * 255

    data.vision_image[:,:,2] = threshed      #multiply the binary threshed image by 255 converting to blue
    data.vision_image[:,:,0] = obs_map       #multiply the binary threshed image by 255 converting to red    

    # erase the top of the image as we can not reliably do prediction close the horizon
    # 8/10
    threshed[0:(int)(threshed.shape[0] * 8.0 / 10.0), :] = 0    
    
    # 5) Convert map image pixel values to rover-centric coords
    # 6) Convert rover-centric pixel values to world coordinates
    xpix, ypix = rover_coords(threshed)
    xpix_world, ypix_world = pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale)
    
    xpix_obs, ypix_obs = rover_coords(obs_map)
    xpix_obs_world, ypix_obs_world = pix_to_world(xpix_obs, ypix_obs, xpos, ypos, yaw, world_size, scale)
        
        # Add the warped image in the upper right hand corner
    output_image[0:img.shape[0], img.shape[1]:] = warped

    # 7) Update Rover worldmap (to be displayed on right side of screen)
    # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
    #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
    #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
    data.worldmap[ypix_world, xpix_world, 2] = 255
    data.worldmap[ypix_obs_world, xpix_obs_world, 0] = 255
    nav_pix = data.worldmap[:,:,2] > 0
    data.worldmap[nav_pix, 0] = 0
    
    #call function to look for rocks
    rock_map = find_rocks(warped, levels=(110,110,50))
    # if rock_map.any():
    #     rock_x, rock_y = rover_coords(rock_map)
    #     rock_x_world, rock_y_world = pix_to_world(rock_x, rock_y, xpos, ypos, yaw, world_size, scale)
    #     data.worldmap[rock_y_world, rock_x_world, :] = 255

    if rock_map.any():
        rock_x, rock_y = rover_coords(rock_map)
        rock_x_world, rock_y_world = pix_to_world(rock_x, rock_y, data.pos[0], data.pos[1], data.yaw, world_size, scale)
        rock_dist, rock_ang = to_polar_coords(rock_x, rock_y)
        data.rock_x_world = rock_x_world
        data.rock_y_world = rock_y_world
        data.rock_dist = rock_dist
        data.rock_ang = rock_ang
        rock_idx = np.argmin(rock_dist)
        rock_xcen = rock_x_world[rock_idx]
        rock_ycen = rock_y_world[rock_idx]
        data.worldmap[rock_ycen, rock_xcen, 1] = 255
        data.vision_image[:,:,1] = rock_map * 255
    else:
        data.vision_image[:,:,1] = 0
        data.rock_dist = 0 #will be set to distance from rock in perception.py
        data.rock_ang = np.array([])

        # Overlay worldmap with ground truth map
    map_add = cv2.addWeighted(data.worldmap, 1, data.ground_truth, 0.5, 0)

    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles
    dist, angles = to_polar_coords(xpix, ypix)
    data.nav_dists = dist
    data.nav_angles = angles


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
    # 2) Apply perspective transform
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image

    # 5) Convert map image pixel values to rover-centric coords
    # 6) Convert rover-centric pixel values to world coordinates
    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1

    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles
    
 
    process_image(Rover)
    
    return Rover