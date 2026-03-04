from robot import Robot_Sim, PidController
from grid import Grid
# from utils import *
from utils import rotate_point, grid_distance, find_centroid
import math
import numpy as np
import random

def get_wheel_velocities_disrupter(robbie, index):
    """
    Helper function to determine the velocities of the robot's left and right wheels.
    Arguments:
        robbie: instance of the robot
        coord (tuple): coordinate to move to (x,y)
    
    Returns: 
        vr, vl: velocities of the robot's left and right wheels
    """

    current_goal_x = robbie.disrupter_goals[index][robbie.disrupter_states[index]][0]
    current_goal_y = robbie.disrupter_goals[index][robbie.disrupter_states[index]][1]

    # Calculate the desired change in position
    dx_world = current_goal_x - robbie.disrupter_poses[index][0]
    dy_world = current_goal_y - robbie.disrupter_poses[index][1]
    dx_robot, dy_robot = rotate_point(dx_world, dy_world, robbie.disrupter_poses[index][2])
    dist_to_coord = math.sqrt(dx_robot**2 + dy_robot**2)
    
    
    # Using desired linear velocity, set left and right wheel velocity
    # Turn in place first
    angle = math.atan2(dy_robot, dx_robot)
    threshold = 0.1
    
    # Using PID controller for linear and angular velocities
    linear_v = robbie.pid_controller_disrupter.linear_controller((robbie.disrupter_poses[index][0], robbie.disrupter_poses[index][1], robbie.disrupter_poses[index][2]), (current_goal_x, current_goal_y))
    w = robbie.pid_controller_disrupter.angular_controller((robbie.disrupter_poses[index][0], robbie.disrupter_poses[index][1], robbie.disrupter_poses[index][2]), (current_goal_x, current_goal_y))
    if angle < -threshold or angle > threshold:
        linear_v = 0
        
    # Save the wheel velocities to the robot instance to use in Webots
    robbie.v_disrupter[index] = linear_v
    robbie.w_disrupter[index] = w
    
    # Calculate the wheel linear velocities
    vl = (linear_v - robbie.wheel_dist_disrupter / 2 * w) 
    vr = (linear_v + robbie.wheel_dist_disrupter / 2 * w)

    # Cap the wheel velocities to a maximum value but keep the ratio between them
    omega_max = 6.279
    omega_l = vl/robbie.wheel_r_disrupter
    omega_r = vr/robbie.wheel_r_disrupter
    omega_current_max = max(abs(omega_l), abs(omega_r))
    if omega_current_max > omega_max:
        scale = omega_max/omega_current_max
        vl = vl*scale
        vr = vr*scale

    # vl = 0
    # vr = 0

    return vr, vl

def update_disrupters(robbie, grid):
    # Update the disrupters' goal positions and velocities
    for index, disrupter_pose in enumerate(robbie.disrupter_poses):
        current_goal_x = robbie.disrupter_goals[index][robbie.disrupter_states[index]][0]
        current_goal_y = robbie.disrupter_goals[index][robbie.disrupter_states[index]][1]
        # If the disrupter is close enough to its goal, switch its state
        # print(f"disrupter_goals: {robbie.disrupter_goals}")
        # print(f"current_goal_x: {current_goal_x}, current_goal_y: {current_goal_y}")
        if grid_distance(disrupter_pose[0], disrupter_pose[1], current_goal_x, current_goal_y) <= 2:
            robbie.disrupter_states[index] = not robbie.disrupter_states[index]
        
        # If a disrupter is close to the robbie, then stop the disrupter to avoid collision
        if grid_distance(robbie.x, robbie.y, disrupter_pose[0], disrupter_pose[1]) < 10: #math.ceil(4.2 + 0.925):
            robbie.v_disrupter[index] = 0
            robbie.w_disrupter[index] = 0
            robbie.vl_disrupter[index] = 0
            robbie.vr_disrupter[index] = 0
        else:
            # Calculate the wheel velocities for the disrupter
            robbie.vr_disrupter[index], robbie.vl_disrupter[index] = get_wheel_velocities_disrupter(robbie, index)

        # print(robbie.vr_disrupter[index], robbie.vl_disrupter[index])
