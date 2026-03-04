import math
import numpy as np
from detect_circle import detect_circle

# Instructions:
# Step 1: Review the `detect_circle` function to infer and detect a circle in the image 
#         and compute its angle.
# Step 2: Explore the LiDAR data corresponding to the detected object. Investigate 
#         how to classify the object as either a sphere or a disk. You may reuse 
#         your code from `camera_only.py`.

def camera_and_lidar_calculation(image, camera_fov, object_diameter, lidar_data):
    """
    Performs object detection and classification by combining data from a camera and a LiDAR sensor.

    Args:
        image: The input image captured by the camera.
        camera_fov: The field of view of the camera in radians.
        object_diameter: The expected diameter of the object in meters.
        lidar_data: An array representing LiDAR distance data indexed by angle 
                                  in degrees, where each element corresponds to the distance 
                                  at a specific angle.

    Returns:
        lidar_distance: The distance to the detected object from the LiDAR sensor in meters.
        object_shape: A string indicating the shape of the detected object ("sphere" or "disk").
    """

    ###########################################################################
    # TODO: Student code begins
    
    # Calculate the focal_length
    image_width = 640
    focal_length = image_width / (2 * np.tan(camera_fov / 2))
    
    # Camera Position
    camera_position = [0.03, 0, 0.028]     
         
    # Detect cicle in the image.
    circles = detect_circle(image)
    
    # Extract the data from the circles variable
    u = circles[0][0]
    v = circles[0][1]
    radius = circles[0][2]

    # Obtain parameters in the image
    u0 = image_width / 2
    object_diameter_in_image = 2 * radius
    x = u - u0

    # Calculate depth
    depth = focal_length * (object_diameter / object_diameter_in_image) + camera_position[0]

    # Calculate relative angle with trigonometry.
    angle = np.arctan(x / focal_length)
    cam_to_object_x = depth * np.sin(angle)
    aux = depth - (np.cos(angle) / focal_length)
    diff_y = aux * np.cos(angle)
    object_x = camera_position + cam_to_object_x
    object_y = focal_length + diff_y
    
    relative_angle = np.arctan(object_x / object_y)
    
    # Change from radians to degrees, the value in degrees will be the
    # index of the lidar_distance that points to the sphere/disk
    angle_degrees = angle * (180 / np.pi)
    lidar_angle = round(angle_degrees)
    if lidar_angle == 360:
        lidar_angle = 0
    reminder = abs(angle_degrees - lidar_angle)
    
    # Finally we calculate a more exact distance with the 2 closest lidar
    # distances measurements, weighted with the angle reminder of the 
    # rounding function
    lidar_distance_1 = lidar_data[lidar_angle]
    if reminder >= 0.5:
        lidar_distance_2 = lidar_data[lidar_angle - 1]
        lidar_distance = lidar_distance_1 * reminder + lidar_distance_2 * (1-reminder)
    else:
        lidar_distance_2 = lidar_data[lidar_angle + 1]
        lidar_distance = lidar_distance_1 * (1-reminder) + lidar_distance_2 * reminder 
    
    # Depending on distance we decide the angle difference we will use
    value = 4
    
    print(lidar_data[lidar_angle - value], lidar_distance, lidar_data[lidar_angle + value])
    # Depending of the previous and following lidar_distances we can
    # infere the shape
    if round(lidar_data[lidar_angle - value], 2) == round(lidar_data[lidar_angle + value], 2):
        object_shape = "disk"
    else:
        object_shape = "sphere"

    # Student code ends
    ###########################################################################

    return lidar_distance, object_shape
