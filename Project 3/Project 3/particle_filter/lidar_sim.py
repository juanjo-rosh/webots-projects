import math
import numpy as np
from utils import *
from wall import Wall

class LidarSim:
    # Constructor
    def __init__(self, walls:list[Wall], max_range:float, n_rays:float):
        self.walls = walls
        self.max_range = max_range
        self.n_rays = n_rays
        self.resolution = int(360/n_rays)
        self.measurements = math.inf*np.ones(self.resolution)

    # Simulate the lidar sensor reading
    def read(self, pose:SE2) -> np.ndarray:
        '''
        Simulate the lidar sensor readings given the current pose of the robot.

        Parameters:
        pose (SE2): The current pose of the robot, represented as an SE2 object, 
        which includes the x and y coordinates and the heading (orientation) of 
        the robot.

        Returns:
        np.ndarray: An array of simulated lidar measurements, where each element 
        represents the distance to the nearest wall for a specific lidar ray.

        Steps:
        1. Iterate through each lidar ray:
           - For each ray, calculate the angle based on the robot's heading and 
           the resolution of the lidar.
           - Determine the endpoint of the lidar ray based on the maximum range 
           and the calculated angle.

        2. Check for intersections with walls:
           - For each wall, check if the lidar ray intersects with the wall 
           using the line_rectangle_intersect function.
           - If an intersection is detected, calculate the intersection points 
           between the lidar ray and the edges of the wall.
           - Calculate the distances from the robot to these intersection points.

        3. Find the minimum distance:
           - Among all intersection points, find the minimum distance and update 
           the measurements array for the corresponding ray.

        4. Return the measurements:
           - Return the array of simulated lidar measurements.
        '''

        # Reset the measurements
        self.measurements = math.inf*np.ones(self.n_rays) # Webots lidar sensor returns inf for no detection
        
        ######### START STUDENT CODE #########
        # Hint - You may find the following functions in utils.py useful: 
        # line_rectangle_intersect, line_segment_intersect, line_intersection, distance_between_points

        # Step 1: Iterate through each lidar ray.
        for i in range(self.n_rays):
            angle = pose.h + math.radians(i * self.resolution + (self.resolution / 2))
            
            start_point = Point(pose.x, pose.y)
            end_point = Point(pose.x + self.max_range * math.cos(angle),
                              pose.y + self.max_range * math.sin(angle))
            
            # Initialize intersection_points for this ray (accumulate intersections from all walls).
            intersection_points = []
            
            # Step 2: Check intersections with each wall.
            for wall in self.walls:
                detect = line_rectangle_intersect(start_point, end_point, wall.pose, wall.dimensions)
                if detect:
                    # Create points for each wall corner.
                    p_top_left = Point(wall.top_left[0], wall.top_left[1])
                    p_top_right = Point(wall.top_right[0], wall.top_right[1])
                    p_bottom_left = Point(wall.bottom_left[0], wall.bottom_left[1])
                    p_bottom_right = Point(wall.bottom_right[0], wall.bottom_right[1])
                    
                    # Compute intersections for each wall edge.
                    intersection_up = line_intersection(start_point, end_point, p_top_left, p_top_right)
                    intersection_down = line_intersection(start_point, end_point, p_bottom_left, p_bottom_right)
                    intersection_left = line_intersection(start_point, end_point, p_top_left, p_bottom_left)
                    intersection_right = line_intersection(start_point, end_point, p_top_right, p_bottom_right)
                    
                    if intersection_up is not None:
                        intersection_points.append(intersection_up)
                    if intersection_down is not None:
                        intersection_points.append(intersection_down)
                    if intersection_left is not None:
                        intersection_points.append(intersection_left)
                    if intersection_right is not None:
                        intersection_points.append(intersection_right)
            
            # Step 3: Determine the minimum distance from start_point to any intersection.
            min_distance = math.inf
            for p in intersection_points:
                d = distance_between_points(start_point, p)
                if d < min_distance:
                    min_distance = d 
            
            # Step 4: Update the measurement for the current ray.
            self.measurements[i] = min_distance

        ########## END STUDENT CODE ##########

        return self.measurements
