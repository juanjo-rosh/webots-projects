import numpy as np
import math
from detect_circle import detect_circle

# Instructions:
# Review the `detect_circle` function to infer and detect a circle in the image and compute its angle.

def vision_only_distance_calculation(image, camera_fov, object_diameter):
    """
    This function performs object detection and calculates the depth and angle of the detected circle from a camera sensor.

    Args:
        image: The input image from the camera
        camera_fov: The field of view of the camera in radians.
        object_diameter: The expected diameter of the object in meters.

    Returns:
        depth: The depth to the detected object from camera depth estimation in meters.
        angle: the angle of the detected circle in radians.
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
    
    # Student code ends
    ###########################################################################

    return depth, angle