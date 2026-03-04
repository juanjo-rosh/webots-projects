from robot import Robot_Sim, PidController
from grid import Grid
# from utils import *
from utils import rotate_point, grid_distance, find_centroid, separate_frontiers
import math
import numpy as np
import random

def get_wheel_velocities(robbie, coord):
    """
    Helper function to determine the velocities of the robot's left and right wheels.
    Arguments:
        robbie: instance of the robot
        coord (tuple): coordinate to move to (x,y)
    
    Returns: 
        vr, vl: velocities of the robot's left and right wheels
    """

    # Calculate the desired change in position
    dx_world = coord[0] - robbie.x
    dy_world = coord[1] - robbie.y
    dx_robot, dy_robot = rotate_point(dx_world, dy_world, robbie.h)
    dist_to_coord = math.sqrt(dx_robot**2 + dy_robot**2)
    
    
    # Using desired linear velocity, set left and right wheel velocity
    # Turn in place first
    angle = math.atan2(dy_robot, dx_robot)
    threshold = 0.1
    
    # Using PID controller for linear and angular velocities
    linear_v = robbie.pid_controller.linear_controller((robbie.x, robbie.y, robbie.h), coord)
    w = robbie.pid_controller.angular_controller((robbie.x, robbie.y, robbie.h), coord)
    if angle < -threshold or angle > threshold:
        linear_v = 0
        
    # Save the wheel velocities to the robot instance to use in Webots
    robbie.v = linear_v
    robbie.w = w
    
    # Calculate the wheel linear velocities
    vl = (linear_v - robbie.wheel_dist / 2 * w) 
    vr = (linear_v + robbie.wheel_dist / 2 * w)

    # Cap the wheel velocities to a maximum value but keep the ratio between them
    omega_max = 6.279
    omega_l = vl/robbie.wheel_r
    omega_r = vr/robbie.wheel_r
    omega_current_max = max(abs(omega_l), abs(omega_r))
    if omega_current_max > omega_max:
        scale = omega_max/omega_current_max
        vl = vl*scale
        vr = vr*scale


    return vr, vl


def get_neighbors(cell):
    """
    Get neighbors of a given cell
    """
    return [
        (cell[0]+1, cell[1]),
        (cell[0]-1, cell[1]),
        (cell[0], cell[1]+1),
        (cell[0], cell[1]-1)
    ]


def frontier_planning(robbie: Robot_Sim, grid: Grid):
    """
        Function for defining frontier planning.

        Arguments:
            robbie: instance of the robot
            grid: instance of the grid

        Returns:
            robbie: 'updated' instance of the robot
            OPTIONAL: robbie.next_coord: new destination coordinate

        Notes:
            The lecture notes should provide you with an ample description of frontier planning.
            You will also find many of the functions declared in 'grid.py' and 'utils.py' useful.

        Hints: 
            You may find these helper functions useful for implementing frontier_planning(): 
                Grid.is_free(),  Grid.is_in() 
                Robot_Sim.explored_cells 
                get_neighbors()  [exploration.py] 
                find_centroid()  [utils.py] 
                grid_distance()  [utils.py]
                separate_frontiers()  [utils.py]
    """
    frontier_cells = []
    ## TODO: STUDENT CODE START ##

    # Get all frontier cells based on definition: frontiers cells are
        # 1. unexplored cells
        # 2. adjacent to explored cells
        # 3. in free space and within boundaries
        
    for cell in robbie.explored_cells:
        if grid.is_free(cell[0], cell[1]):
            for n in get_neighbors(cell):
                if (n not in robbie.explored_cells and
                    grid.is_free(n[0], n[1]) and
                    grid.is_in(n[0], n[1]) and
                    n not in frontier_cells):
                    frontier_cells.append(n)
                    
    # Separate the adjacenct cells into separate frontiers
    frontiers = separate_frontiers(frontier_cells, grid)
    
    # Compute the centroids of the frontiers
    centroids = []
    for frontier in frontiers:
        if frontier:
            centroids.append((find_centroid(frontier), frontier))
    
    # Sort the centroids based on 
        # 1. their distances to the robbie's current position
        # 2. size of the frontier (number of grid cells in the frontier) that the centroid belongs to
    current_pos = (robbie.x, robbie.y)
    w1 = 1  # weight for distance
    w2 = 1  # weight for frontier size

    centroids.sort(key=lambda c: w1 * grid_distance(current_pos[0], current_pos[1], c[0][0], c[0][1]) - w2 * len(c[1]))
    
    # Choose the first centroid from the sorted lsit which is not same as robot's current position and is not in obstacle
    
    for centroid, _ in centroids:
        if (centroid != current_pos and
            grid.is_free(centroid[0], centroid[1])):
            robbie.next_coord = centroid
            break
            
        # In case where no centroid is suitable, pick a random point from the frontier as the robot's next_coord
        else:
            if frontier_cells:
                robbie.next_coord = random.choice(frontier_cells)
                break
            else:
                robbie.next_coord = current_pos
                break
            
    
    ## STUDENT CODE END ##
    #################################################
    grid.centroid = robbie.next_coord
    return robbie, robbie.next_coord


def exploration_state_machine(robbie, grid):
    """
    Use frontier planning, or another exploration algorithm, to explore the grid.

    Arguments:
        robbie: instance of the robot
        grid: instance of the grid

    Returns: 
        robbie: 'updated' instance of the robot

    Notes:
        Robot is considered as Point object located at the center of the traingle. 
        Robot explores the map in the discretized space
        You may use the 'rrt' function (see grid.py) to find a new path whenever the robot encounters an obstacle.
        Please note that the use of rrt slows down your code, so it should be used sparingly.
        The 'get_wheel_velocities' functions is useful in setting the robot's velocities.
        You will also find many of the functions declared in 'grid.py' and 'utils.py' useful.
        Feel free to create other helper functions (in this file) as necessary.

    Alert:
        In this part, the task is to let the robot find all markers by exploring the map,
        which means using 'grid.markers' will lead  cause zero point on GraderScope.

    """
    # Initialize PID controller
    
    linear_kp = 1.0
    linear_ki = 0.0
    linear_kd = 0.0
    angular_kp = 5.0
    angular_ki = 0.0
    angular_kd = 0.0

    if not hasattr(robbie, 'pid_controller'):
        robbie.pid_controller = PidController(linear_kp, linear_ki, linear_kd, angular_kp, angular_ki, angular_kd)
    use_rrt = False

    # Sensing: Get the free space in robot's current FOV
    free_cells = robbie.get_free_cells_in_fov(grid)
    
    ### TODO: STUDENT CODE START ###

    # Planning: If you do not know robbie's next coordinate or have already reached it (current position is close enough to next_coord), 
    # or if the next_coord is not in free space (need to check again here to account for dynamic obstacles),
    # run your choice of exploration to get robbie's next coordinate
    current_pos = (robbie.x, robbie.y)
    threshold_dist = 7
    
    if (not hasattr(robbie, 'next_coord') or
        robbie.next_coord is None or
        grid_distance(current_pos[0], current_pos[1], robbie.next_coord[0], robbie.next_coord[1]) < threshold_dist or
        not grid.is_free(robbie.next_coord[0], robbie.next_coord[1]) or
        not grid.is_in(robbie.next_coord[0], robbie.next_coord[1])):
        """
        if len(robbie.path) != 0 and not grid.is_collision_with_obstacles(current_pos, robbie.next_coord):
            next_target = robbie.path.pop(0).coord
            robbie.next_coord = next_target
        else:
            robbie, robbie.next_coord = frontier_planning(robbie, grid)
        """
        robbie, robbie.next_coord = frontier_planning(robbie, grid)
        
    # If moving to next coordinate results in a collision, then perform RRT and set that as the next coord. 
 
    while grid.is_collision_with_obstacles(current_pos, robbie.next_coord):
        robbie.path = grid.rrt(current_pos, robbie.next_coord)[1:]
        if len(robbie.path) != 0:
            next_target = robbie.path.pop(0).coord
            robbie.next_coord = next_target
        if grid_distance(current_pos[0], current_pos[1], robbie.next_coord[0], robbie.next_coord[1]) < threshold_dist:
            break


        
    # Now that you know the next coordinate, set Robbie's wheel velocities
    
    if (hasattr(robbie, 'next_coord') and robbie.next_coord is not None):
        robbie.vr, robbie.vl = get_wheel_velocities(robbie, robbie.next_coord)
    

    ### STUDENT CODE END ###
    return robbie