import random

from grid import *
from utils import *
import copy

class Robot_Sim(object):

    # data members
    x = "X coordinate in world frame"
    y = "Y coordinate in world frame"
    h = "Heading angle in world frame in degree. h = 0 when robot's head (camera) points to positive X"
    wheel_r = "Radius of wheels of robot"
    wheel_dist = "Distance between wheels of robot"
    TIMESTEP = "timestep"
    path = 'rrt path'

    # functions members
    def __init__(self, x, y, heading=None, wheel_dist=0.052, wheel_r = 0.0205): #wheel_dist=0.5, wheel_r = 1 
        if heading is None:
            heading = random.uniform(0, 360)
        self.__x = x
        self.__y = y
        self.__h = heading % 360
        self.scale = 0.04
        self.wheel_dist = wheel_dist/self.scale
        # self.__TIMESTEP = 2
        self.__TIMESTEP = 0.064
        
        self.wheel_r = wheel_r/self.scale
        # Grid cells already explored by the robot
        self.explored_cells = {(x, y)}

        self.next_coord = None
        self.path = []
        self.markers_found_or_picked = []
        self.curr_marker = None


    def __repr__(self):
        return "(x = %f, y = %f, heading = %f deg)" % (self.__x, self.__y, self.__h)
    
    @property
    def TIMESTEP(self):
        return self.__TIMESTEP
        
    @property
    def x(self):
        return self.__x
    
    @property
    def y(self):
        return self.__y
    
    @property
    def h(self):
        return self.__h % 360
    
    @property
    def xy(self):
        return self.__x, self.__y

    @property
    def xyh(self):
        return self.__x, self.__y, self.__h
    

    def chose_random_heading(self):
        return random.uniform(0, 360)
    
    def init_disrupter(self, grid, disrupter_poses, disrupter_goals):
        # for disrupter_position in disrupter_positions:
        #     self.disrupter_x ,self.disrupter_y = disrupter_position
        self.disrupter_poses = disrupter_poses # list of lists (x, y, yaw)
        self.disrupter_goals = disrupter_goals # list of tuples (x, y)
        self.disrupter_states = [0]*len(disrupter_poses)
        self.wheel_dist_disrupter = 0.2718/self.scale
        self.wheel_r_disrupter = 0.031/self.scale
        self.pid_controller_disrupter = PidController(1, 0, 0, 5, 0, 0)
        self.v_disrupter = [0]*len(disrupter_poses)
        self.w_disrupter = [0]*len(disrupter_poses)
        self.vl_disrupter = [0]*len(disrupter_poses)
        self.vr_disrupter = [0]*len(disrupter_poses)

        # Find all cells occupied by disrupters
        self.disrupter_cells = []
        for disrupter_pose in disrupter_poses:
            disrupter_cell = self.get_cells_of_disrupter(grid, disrupter_pose[0:2])
            self.disrupter_cells = list(set(self.disrupter_cells) | set(disrupter_cell))
        grid.occupied_with_disrupters = copy.deepcopy(grid.occupied)
        for cell in self.disrupter_cells:
            grid.occupied_with_disrupters.append(cell)
            # grid.occupied.append(cell)

    def get_cells_in_fov(self, grid, dist=10):
        """ Get list of grid cells that are in FOV of robot

            Arguments:
            grid -- grid to list cells from
            dist -- range of FOV

            Return: List of visible grid cells
        """
        block_list = []
        r_x, r_y = self.__x, self.__y
        
        for x in range(math.floor(r_x - dist), math.ceil(r_x + dist + 1)):
                for y in range(math.floor(r_y - dist), math.ceil(r_y + dist + 1)):
                    x_dis = r_x - x - 1 if x <= r_x else r_x - x
                    y_dis = r_y - y - 1 if y <= r_y else r_y - y
                    if math.sqrt(x_dis**2 + y_dis**2) > dist:
                        continue
                    if grid.is_in(x, y) and (x != self.__x or y != self.__y):
                        block_list.append((x, y))
                        self.explored_cells.add((x, y))
        return block_list
    
    
    def get_cells_of_disrupter(self, grid, disrupter_position, dist=4.2):
        """ Get list of grid cells that are overlapping with disrupter

            Arguments:
            grid -- grid to list cells from
            dist -- range of FOV

            Return: List of visible grid cells
        """
        block_list = []
        r_x, r_y = disrupter_position
        
        for x in range(math.floor(r_x - dist), math.ceil(r_x + dist + 1)):
                for y in range(math.floor(r_y - dist), math.ceil(r_y + dist + 1)):
                    x_dis = r_x - x - 1 if x <= r_x else r_x - x
                    y_dis = r_y - y - 1 if y <= r_y else r_y - y
                    if math.sqrt(x_dis**2 + y_dis**2) > dist:
                        continue
                    if grid.is_in(x, y) and (x != self.__x or y != self.__y):
                        block_list.append((x, y))
                        # self.explored_cells.add((x, y))
        return block_list


    def get_obstacles_in_fov(self, grid, dist=10):
        """ Get list of obstacles that are in FOV of robot

            Arguments:
            grid -- grid to list cells from
            dist -- range of FOV

            Return: List of visible cells occupied by obstacles
        """
        obstacle_list = []
        visible_grid_cells = self.get_cells_in_fov(grid, dist)
        for (x, y) in visible_grid_cells:
            if grid.is_in(x, y) and grid.is_occupied(x, y):
                obstacle_list.append((x, y))
        return obstacle_list
    

    def get_free_cells_in_fov(self, grid, dist=10):
        """ Get list of free grid cells that are in FOV of robot

            Arguments:
            grid -- grid to list cells from
            dist -- range of FOV

            Return: List of visible cells that are free
        """
        free_cells = []
        obstacle_list = self.get_obstacles_in_fov(grid, dist)
        visible_grid_cells = self.get_cells_in_fov(grid, dist)

        for (x, y) in visible_grid_cells:
            if (x, y) in obstacle_list:
                continue
            free_cells.append((x, y))
        return free_cells
    

    def read_marker_around(self, grid, dis=10):
        """ 
        Get list of markers in robot's FOV
        Arguments:
            grid -- grid to list cells from
            dist -- range of FOV

        Return: List of markers around
        """
        marker_list = []
        free_block = set(self.get_cells_in_fov(grid, dis))
        for marker in grid.markers:
            m_x, m_y, m_h = marker
            if (m_x, m_y) in free_block:
                marker_list.append((m_x, m_y, m_h))
        return marker_list

    def move_diff_drive(self, grid, vl, vr, dt):
        """ Move the robot with a steering angle and diff drive forward.
            Note that the distance between the wheels is 0.5

            Arguments:
            dvl -- velocity to set of left wheel
            dvr -- velocity to set of right wheel

            No return
        """
        # v = (vl+vr) * self.wheel_r/2
        # w = (vr-vl) * self.wheel_r/self.wheel_dist
        v = (vl+vr)/2
        w = (vr-vl)/self.wheel_dist
        self.__h += math.degrees(w)*dt
        
        h_rad = math.radians(self.__h)
        dx = v * math.cos(h_rad) * dt
        dy = v * math.sin(h_rad) * dt

        # Check if theres a collision along path
        m, c = find_line((self.__x, self.__y), (self.__x+dx, self.__y+dy))
        if m == math.inf:
            m,c = find_line((self.__x+0.01, self.__y), (self.__x+dx, self.__y+dy))

        x_range = [self.__x + .1 * i for i in range(1, math.floor(dx / .1))]
        x_range = [self.__x] + x_range + [self.__x+dx]
        # print("x_range: ", x_range)
        for xi in x_range:
            yi = m * xi + c
            yi = max(yi, 0)
            xi = max(xi, 0)
            if not grid.is_free(math.floor(xi), math.floor(yi)):
                raise Exception(f"grid ({math.floor(xi)}, {math.floor(yi)}) isn't free error")
        self.__x += dx
        self.__y += dy
        
        self.get_cells_in_fov(grid)

        return (self.__x, self.__y, self.__h)
    
    def move_diff_drive_no_update(self, grid, vl, vr, dt):
        """ Move the robot with a steering angle and diff drive forward.
            Note that the distance between the wheels is 0.5

            Arguments:
            dvl -- velocity to set of left wheel
            dvr -- velocity to set of right wheel

            No return
        """
        # v = (vl+vr) * self.wheel_r/2
        # w = (vr-vl) * self.wheel_r/self.wheel_dist
        v = (vl+vr)/2
        w = (vr-vl)/self.wheel_dist
        # self.__h += math.degrees(w)*dt
        
        h_rad = math.radians(self.__h)
        dx = v * math.cos(h_rad) * dt
        dy = v * math.sin(h_rad) * dt

        # Check if theres a collision along path
        m, c = find_line((self.__x, self.__y), (self.__x+dx, self.__y+dy))
        if m == math.inf:
            m,c = find_line((self.__x+0.01, self.__y), (self.__x+dx, self.__y+dy))

        x_range = [self.__x + .1 * i for i in range(1, math.floor(dx / .1))]
        x_range = [self.__x] + x_range + [self.__x+dx]
        # print("x_range: ", x_range)
        for xi in x_range:
            yi = m * xi + c
            yi = max(yi, 0)
            xi = max(xi, 0)
            if not grid.is_free(math.floor(xi), math.floor(yi)):
                raise Exception(f"grid ({math.floor(xi)}, {math.floor(yi)}) isn't free error")
        # self.__x += dx
        # self.__y += dy
        
        # self.get_cells_in_fov(grid)

        return (self.__x, self.__y, self.__h)
    
    def move_diff_drive_disrupters(self, grid):
        """ Move the robot with a steering angle and diff drive forward.
            Note that the distance between the wheels is 0.5

            Arguments:
            dvl -- velocity to set of left wheel
            dvr -- velocity to set of right wheel

            No return
        """
        for index in range(len(self.disrupter_poses)):
            vl = self.vl_disrupter[index]
            vr = self.vr_disrupter[index]
            dt = self.TIMESTEP

            v = (vl+vr)/2
            w = (vr-vl)/self.wheel_dist_disrupter
            self.disrupter_poses[index][2] += math.degrees(w)*dt
            
            h_rad = math.radians(self.disrupter_poses[index][2])
            dx = v * math.cos(h_rad) * dt
            dy = v * math.sin(h_rad) * dt

            self.disrupter_poses[index][0] += dx
            self.disrupter_poses[index][1] += dy
        
        # Find all cells occupied by disrupters
        self.disrupter_cells = []
        for disrupter_pose in self.disrupter_poses:
            disrupter_cell = self.get_cells_of_disrupter(grid, disrupter_pose[0:2])
            self.disrupter_cells = list(set(self.disrupter_cells) | set(disrupter_cell))
        grid.occupied_with_disrupters = copy.deepcopy(grid.occupied)
        for cell in self.disrupter_cells:
            grid.occupied_with_disrupters.append(cell)
            # grid.occupied.append(cell)


    def move_diff_drive_disrupters_no_update(self, grid):
        """ Move the robot with a steering angle and diff drive forward.
            Note that the distance between the wheels is 0.5

            Arguments:
            dvl -- velocity to set of left wheel
            dvr -- velocity to set of right wheel

            No return
        """
        for index in range(len(self.disrupter_poses)):
            vl = self.vl_disrupter[index]
            vr = self.vr_disrupter[index]
            dt = self.TIMESTEP

            v = (vl+vr)/2
            w = (vr-vl)/self.wheel_dist_disrupter
            # self.disrupter_poses[index][2] += math.degrees(w)*dt
            
            # h_rad = math.radians(self.disrupter_poses[index][2])
            # dx = v * math.cos(h_rad) * dt
            # dy = v * math.sin(h_rad) * dt

            # self.disrupter_poses[index][0] += dx
            # self.disrupter_poses[index][1] += dy

    
    def update_robot_position(self, grid, tracking_data):
        # # x, y = grid.cont_to_discrete(x_webots, y_webots)
        # x = (x_webots + grid.cont_width/2)/grid.grid_size
        # y = (grid.cont_height/2 - y_webots)/grid.grid_size
        # # print(f"update_robot_position: {x}, {y}, {h_webots}")
        # self.__x = x
        # self.__y = y
        # self.__h = math.degrees(-h_webots)
        # print(tracking_data)

        self.__x = (tracking_data["e-puck"]['x'] + grid.cont_width/2)/grid.grid_size
        self.__y = (grid.cont_height/2 - tracking_data["e-puck"]['y'])/grid.grid_size
        self.__h = math.degrees(-tracking_data["e-puck"]['h'])

        self.disrupter_poses[0][0] = (tracking_data["Disrupter1"]['x'] + grid.cont_width/2)/grid.grid_size
        self.disrupter_poses[0][1] = (grid.cont_height/2 - tracking_data["Disrupter1"]['y'])/grid.grid_size
        self.disrupter_poses[0][2] = math.degrees(-tracking_data["Disrupter1"]['h'])

        self.disrupter_poses[1][0] = (tracking_data["Disrupter2"]['x'] + grid.cont_width/2)/grid.grid_size
        self.disrupter_poses[1][1] = (grid.cont_height/2 - tracking_data["Disrupter2"]['y'])/grid.grid_size
        self.disrupter_poses[1][2] = math.degrees(-tracking_data["Disrupter2"]['h'])

        self.get_cells_in_fov(grid)

        # Find all cells occupied by disrupters
        self.disrupter_cells = []
        for disrupter_pose in self.disrupter_poses:
            disrupter_cell = self.get_cells_of_disrupter(grid, disrupter_pose[0:2])
            self.disrupter_cells = list(set(self.disrupter_cells) | set(disrupter_cell))
        grid.occupied_with_disrupters = copy.deepcopy(grid.occupied)
        for cell in self.disrupter_cells:
            grid.occupied_with_disrupters.append(cell)
            # grid.occupied.append(cell)



class PidController():
    """
    A class to represent a PID controller.
    """
    def __init__(self, linear_kp, linear_ki, linear_kd, angular_kp, angular_ki, angular_kd):
        self.linear_error = 0
        self.previous_linear_error = 0
        self.integral_linear_error = 0

        self.angular_error = 0
        self.integral_angular_error = 0
        self.previous_angular_error = 0
        
        self.stopped = False

        # TODO: tune the gains for this PID controller
    
        # Set PID gains (kp, kd, ki) for linear term
        self.linear_kp = linear_kp  # Proportional gain
        self.linear_ki = linear_ki # Integral gain
        self.linear_kd = linear_kd  # Derivative gain
    
        # Set PID gains (kp, kd, ki) for angular term
        self.angular_kp = angular_kp  # Proportional gain
        self.angular_ki = angular_ki # Integral gain
        self.angular_kd = angular_kd  # Derivative gain        
    

    def compute_angular_error(self, current_pose, next_waypoint):
        x_robot, y_robot, theta_robot = current_pose
        dx_world = next_waypoint[0] - current_pose[0]
        dy_world = next_waypoint[1] - current_pose[1]
        dx_robot, dy_robot = rotate_point(dx_world, dy_world, theta_robot)
        angular_error = math.atan2(dy_robot, dx_robot)

        return angular_error
    
    def compute_linear_error(self, current_pose, goal_point):
        x_robot, y_robot, theta_robot = current_pose
        dx_world = goal_point[0] - current_pose[0]
        dy_world = goal_point[1] - current_pose[1]

        dx_robot, dy_robot = rotate_point(dx_world, dy_world, theta_robot)
        dist_to_coord = math.sqrt(dx_robot**2 + dy_robot**2)

        return dist_to_coord
    
    def linear_controller(self, pose, goal_point):
        """
        Set the linear velocity based on the robot's current pose and goal_point.
        
        Parameters:
            pose (np.array): Current pose (x, y, theta)
            goal_point (np.array): Goal pose at the end of the trajectory (x, y)

        Returns: linear_velocity (float) 
        """
        #TODO: return the linear velocity based on the robot's current pose and goal_point. 
        self.linear_error = self.compute_linear_error(pose, goal_point)
        self.integral_linear_error += self.linear_error

        v = self.linear_kp * self.linear_error
        v += self.linear_ki * self.integral_linear_error
        v += self.linear_kd * (self.linear_error - self.previous_linear_error)

        self.previous_linear_error = self.linear_error

        return v
    
    def angular_controller(self, pose, waypoint):
        """
        Set the angular velocity based on the robot's current pose and next waypoint.
        
        Parameters:
            pose (np.array): Current pose (x, y, theta)
            waypoint (np.array): Next waypoint pose to navigate to (x, y)

        Returns: angular_velocity (float) 
        """
        #TODO: return the angular velocity based on the robot's current pose and next waypoint.        
        self.angular_error=self.compute_angular_error(pose, waypoint)
        self.integral_angular_error += self.angular_error

        omega = self.angular_kp * self.angular_error
        omega += self.angular_ki * self.integral_angular_error
        omega += self.angular_kd * (self.angular_error - self.previous_angular_error)

        self.previous_angular_error = self.angular_error

        return omega