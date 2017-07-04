import numpy as np
import cv2

# test - 11:18 -

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160), \
                rock_thresh_min=(60,60,0), rock_thresh_max=(255,255,150)):

    # Create an array of zeros same xy size as img
    # obstacle, rock samples, and navigable terrain are on different channels

    color_select_obstacle = np.zeros_like(img[:,:,0])   
    color_select_rock = np.zeros_like(img[:,:,0])
    color_select_nav = np.zeros_like(img[:,:,0])

    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])       
            
    above_thresh_obs = (img[:,:,0] < rgb_thresh[0]) \
                & (img[:,:,1] < rgb_thresh[1]) \
                & (img[:,:,2] < rgb_thresh[2]) 
            
    above_thresh_rock = (img[:,:,0] > rock_thresh_min[0]) & (img[:,:,0] < rock_thresh_max[0]) \
                    & (img[:,:,1] > rock_thresh_min[1]) & (img[:,:,1] < rock_thresh_max[1]) \
                    & (img[:,:,2] > rock_thresh_min[2]) & (img[:,:,2] < rock_thresh_max[2])\
        

    # Index the array of zeros with the boolean array and set to 1
    color_select_obstacle[above_thresh_obs] = 1
    color_select_rock[above_thresh_rock] = 1
    color_select_nav[above_thresh] = 1
    
    # Return the binary image
    return color_select_obstacle, color_select_rock, color_select_nav

# Define a function to convert to rover-centric coordinates
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = np.absolute(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[0]).astype(np.float)
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

    # convert yaw angle from degrees to radians
    yaw_rad = yaw * np.pi/180
    
    # perform rotation matrix
    xpix_rotated = xpix * np.cos(yaw_rad) - ypix * np.sin(yaw_rad)
    ypix_rotated = xpix * np.sin(yaw_rad) + ypix * np.cos(yaw_rad)

    # Return the result  
    return xpix_rotated, ypix_rotated

# Define a function to perform a translation
def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # TODO:
    # Apply a scaling and a translation

    # perform translation and convert to integer since pixel values can't be float
    xpix_translated = np.int_(xpos + xpix_rot / scale)
    ypix_translated = np.int_(ypos + ypix_rot / scale)

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
    
    return warped


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

    image = Rover.img
    dst_size = 10
    bottom_offset = 6

    # 1 - define source and destination points
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[image.shape[1]/2 - dst_size, image.shape[0] - bottom_offset],
                  [image.shape[1]/2 + dst_size, image.shape[0] - bottom_offset],
                  [image.shape[1]/2 + dst_size, image.shape[0] - 2*dst_size - bottom_offset], 
                  [image.shape[1]/2 - dst_size, image.shape[0] - 2*dst_size - bottom_offset],
                  ]) 

    scale = 10

    # 2 - apply perspective transform
    warped = perspect_transform(image, source, destination)

    # 3 - apply color threshold

    rgb_thresh = (160, 160, 160)
    rock_thresh_min = (50, 45, 0)
    rock_thresh_max = (255, 255, 35)

    colorsel_obs, colorsel_rock, colorsel_nav = color_thresh(warped, \
                                rgb_thresh, rock_thresh_min, rock_thresh_max)

    # 4 - update Rover.vision image
    Rover.vision_image[:,:,0] = colorsel_obs * 255
    Rover.vision_image[:,:,1] = colorsel_rock * 255
    Rover.vision_image[:,:,2] = colorsel_nav * 255

    # 5 - convert map image pixel values to rover centric coordinates
    obs_xpix, obs_ypix = rover_coords(colorsel_obs)
    rock_xpix, rock_ypix = rover_coords(colorsel_rock)
    nav_xpix, nav_ypix = rover_coords(colorsel_nav)
    # xpix_obs, ypix_obs, xpix_rock, ypix_rock, xpix_nav, ypix_nav, = rover_coords(colorsel_obs, colorsel_rock, colorsel_nav)

    # 6 - convert rover centric pixel values to world coordinates
    obs_x_world, obs_y_world = pix_to_world(
        obs_xpix, obs_ypix, Rover.pos[0], Rover.pos[1],
        Rover.yaw, Rover.worldmap.shape[0], scale)

    rock_x_world, rock_y_world = pix_to_world(
        rock_xpix, rock_ypix, Rover.pos[0], Rover.pos[1],
        Rover.yaw, Rover.worldmap.shape[0], scale)

    nav_x_world, nav_y_world = pix_to_world(
        nav_xpix, nav_ypix, Rover.pos[0], Rover.pos[1],
        Rover.yaw, Rover.worldmap.shape[0], scale)

    # 7 - update rover worldmap to be displayed
    Rover.worldmap[obs_y_world, obs_x_world, 0] += 1
    Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
    Rover.worldmap[nav_y_world, nav_x_world, 2] += 1

    # 8 - convert rover centric pixel positions to polar coordinates
    Rover.nav_dists, Rover.nav_angles = to_polar_coords(obs_xpix, obs_ypix)

    # print (Rover.pos[0])
    # print (Rover.pos[1])

    # print ("perception_step successful")

    # print (Rover.nav_dists)
    # print (Rover.nav_angles)
    
    
    return Rover




