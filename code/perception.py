import numpy as np
import cv2

#Identifying the navigable terrain.
#Threshold of RGB > 160 does a nice job of identifying the ground pixels.
def find_nav_path(img, thresh = [160,160,160]):
    # Create an array of zeros same xy size as img, but single channel
    masked_img = np.zeros_like(img[:,:,0])
    #Creating a mask for assigning 1 or True, to the pixels that satisfy all three thresholds.
    #All three need to be true, so bitwise & for the single bit arrays.
    mask = (img[:,:,0] > thresh[0]) & (img[:,:,1] > thresh[1]) & (img[:,:,2] > thresh[2])
    #Item assignemt of True through masking to the pixels that satify the above condition.
    masked_img[mask] = 1
    # Return the binary image
    return masked_img

#In a similar fashion, create two more functions for identifying obstacles and the rocks.

#Identifying the rocks.
#R and G make yellow so low-cap the intensity for them at 115 and high-cap the intensity for B at 100.
def find_rocks(img, thresh = [115,115,100]):
    masked_img = np.zeros_like(img[:,:,0])
    mask = (img[:,:,0] > thresh[0]) & (img[:,:,1] > thresh[1]) & (img[:,:,2] < thresh[2])
    masked_img[mask] = 1
    return masked_img

#Identifying the obstacles.
#Threshold of RGB < 160 reverses the threshold to find the obstacles.
def find_obstacles(img, thresh = [160,160,160]):
    masked_img = np.zeros_like(img[:,:,0])
    mask = (img[:,:,0] < thresh[0]) & (img[:,:,1] < thresh[1]) & (img[:,:,2] < thresh[2])
    masked_img[mask] = 1
    return masked_img

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
    
    return warped


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    img = Rover.img

    # 1) Define source and destination points for perspective transform
    dst_size = 5 
    bottom_offset = 6
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[img.shape[1]/2 - dst_size, img.shape[0] - bottom_offset],
                      [img.shape[1]/2 + dst_size, img.shape[0] - bottom_offset],
                      [img.shape[1]/2 + dst_size, img.shape[0] - 2*dst_size - bottom_offset], 
                      [img.shape[1]/2 - dst_size, img.shape[0] - 2*dst_size - bottom_offset],
                      ])
    
    # 2) Apply perspective transform
    warped = perspect_transform(img, source, destination)
    
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    obstacles_img = find_obstacles(warped)
    rocks_img = find_rocks(warped)
    nav_path_img = find_nav_path(warped)
    
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image
    Rover.vision_image[:,:,0] = obstacles_img*255
    Rover.vision_image[:,:,1] = rocks_img*255
    Rover.vision_image[:,:,2] = nav_path_img*255

    # 5) Convert map image pixel values to rover-centric coords
    obstacle_x, obstacle_y = rover_coords(obstacles_img)
    rock_x, rock_y = rover_coords(rocks_img)
    nav_x, nav_y = rover_coords(nav_path_img)

    # 6) Convert rover-centric pixel values to world coordinates
    xpos = Rover.pos[0]
    ypos = Rover.pos[1]
    yaw = Rover.yaw
    world_size = Rover.worldmap.shape[0]
    scale = 10

    obstacle_x_world, obstacle_y_world = pix_to_world(obstacle_x, obstacle_y, xpos, ypos, yaw, world_size, scale)
    rock_x_world, rock_y_world = pix_to_world(rock_x, rock_y, xpos, ypos, yaw, world_size, scale)
    nav_x_world, nav_y_world = pix_to_world(nav_x, nav_y, xpos, ypos, yaw, world_size, scale)

    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
    Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
    Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
    Rover.worldmap[nav_y_world, nav_x_world, 2] += 1

    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles
    Rover.nav_dists, Rover.nav_angles = to_polar_coords(nav_x, nav_y)
    
    return Rover