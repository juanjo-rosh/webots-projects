from controller import Robot
from geometry import SE2, Point
import math
import json
import numpy as np
# from robot_gui import RobotEnv, RobotEnvThread
# from gui import *
import time
import os
import sys

# from grid import Grid
# from gui import GUIWindow
# from robot import Robot_Sim
# from exploration import exploration_state_machine
# from utils import *
# from generate_noise import add_noise, add_offset_noise

TIME_STEP = 64
MAX_SPEED = 6.28

class MoveDisrupter:

    def __init__(self,robot,path,TIME_STEP,MAX_SPEED):
        self.robot = robot
        self.path = path
        self.TIME_STEP = TIME_STEP
        self.MAX_SPEED = MAX_SPEED
        self.step = 1
        self.step_vis = 1
        self.last_step_measured = 0
        self.goal_flag = False

        self.axle_length = 0.052
        self.wheel_radius = 0.0205
        self.sim_to_real_scale = 0.04

        # get a handler to the motors and set target position to infinity (speed control)
        self.leftMotor = robot.getDevice('left wheel motor')
        self.rightMotor = robot.getDevice('right wheel motor')

        # Set initial wheel positions
        self.leftMotor.setPosition(float('inf'))
        self.rightMotor.setPosition(float('inf'))

        # Get position sensors for wheels
        self.left_ps = self.robot.getDevice('left wheel sensor')
        self.right_ps = self.robot.getDevice('right wheel sensor')
        self.left_ps.enable(self.TIME_STEP)
        self.right_ps.enable(self.TIME_STEP)

        self.wheel_position = [self.left_ps.getValue(), self.right_ps.getValue()]
        print("wheel initial positions", self.wheel_position)

        # Get absolute position of robot
        self.gps = self.robot.getDevice('gps')
        self.gps.enable(self.TIME_STEP)

        # Get coordinate system of robot
        self.compass = self.robot.getDevice('compass')
        self.compass.enable(self.TIME_STEP)

        # Initialize the receiver
        self.receiver = self.robot.getDevice("receiver")
        self.receiver.setChannel(1)
        self.receiver.enable(self.TIME_STEP)

        # Initialize motors
        self.leftMotor.setVelocity(0.0)
        self.rightMotor.setVelocity(0.0)
        self.robot.step(TIME_STEP)

    def get_robot_pose(self):
        """
        Transform from translation (x, y, z) and north direction into SE2(x, y, h)
        """
        x, y, _ = self.gps.getValues()
        compass_value = self.compass.getValues()
        h = math.atan2(compass_value[0], compass_value[1])
        return SE2(x, y, h)
        
    def get_motion_info(self):

        dt = (self.step - self.last_step_measured) * TIME_STEP / 1000 #in second
        new_wheel_position = [self.left_ps.getValue(), self.right_ps.getValue()]
        wheel_dis_traveled = [new_wheel_position[0] - self.wheel_position[0], new_wheel_position[1] - self.wheel_position[1]]
        self.wheel_position = new_wheel_position
        omega_l, omega_r = wheel_dis_traveled[0] / dt, wheel_dis_traveled[1] / dt
        self.last_step_measured = self.step
        return omega_l, omega_r, dt

    def move_forward(self,next_coords,speed = 5,min_distance = 0.1):

        print("move forward...")
        while robot.step(TIME_STEP) != -1:
            # Set wheel velocity
            self.rightMotor.setVelocity(speed*self.MAX_SPEED)
            self.leftMotor.setVelocity(speed*self.MAX_SPEED)

            # Stop moving when the robot is within min_distance of the next_coords
            current_dist = math.sqrt((self.get_robot_pose().x-next_coords[0])**2 + (self.get_robot_pose().y-next_coords[1])**2)
            if current_dist <= min_distance:
                # Stop moving by setting robot wheel velocities to 0
                self.leftMotor.setVelocity(0.0)
                self.rightMotor.setVelocity(0.0)
                self.goal_flag = True
                break

        time.sleep(0.0001)

    def turn_in_place(self,expected_heading,min_angle):
        speed = 0.1

        while self.robot.step(TIME_STEP) != -1:
            # Turn clockwise
            if expected_heading < self.get_robot_pose().h:
                self.rightMotor.setVelocity(speed * MAX_SPEED * -1)
                self.leftMotor.setVelocity(speed * MAX_SPEED * 1)
            # Turn counterclockwise
            else:
                self.rightMotor.setVelocity(speed * MAX_SPEED * 1)
                self.leftMotor.setVelocity(speed * MAX_SPEED * -1)

            # Stop turning if robot is at expected heading
            heading_diff = abs(self.get_robot_pose().h - expected_heading)
            if heading_diff < min_angle:
                # Stop turning by setting robot wheel velocities to 0
                self.leftMotor.setVelocity(0.0)
                self.rightMotor.setVelocity(0.0)
                break
            
        time.sleep(0.00001)

    def move_to_next_coord(self, node):
        
        min_distance = 0.1
        min_angle = 0.02 #radians
        expected_heading = math.atan2(node[1]-self.get_robot_pose().y,node[0]-self.get_robot_pose().x)
        self.turn_in_place(expected_heading,min_angle=min_angle)
        self.move_forward(next_coords=node,speed=1,min_distance=min_distance)
        return
    
    def follow_path(self):
        for node in self.path:
            min_distance = 0.1
            min_angle = 0.02 #radians
            # Turn to angle of next coord
            print(node,' turn in place')
            expected_heading = math.atan2(node[1]-self.get_robot_pose().y,node[0]-self.get_robot_pose().x)
            print("Expected heading webots: ", expected_heading)
            self.turn_in_place(expected_heading,min_angle=min_angle)
            #  Move towards point
            self.move_forward(next_coords=node,speed=1,min_distance=min_distance)
            
        print('At the goal!')

        return
    
    def receive_veloticy_command(self):
        v = 0
        w = 0

        if self.receiver.getQueueLength() > 0:
            data = self.receiver.getString()#.decode("utf-8")

            # Discard all packets in the queue
            while self.receiver.getQueueLength() != 0:
                self.receiver.nextPacket()
            
            decoded_data = json.loads(data)
            # self.robot.velocity_command = decoded_data[self.robot_name]
            # print(self.robot.velocity_command)
            v = decoded_data[self.robot_name]['v']
            w = decoded_data[self.robot_name]['w']
            # print("v, w: ", v, w)

        return v, w

    def move_motors(self, v, w):
        # Convert the robbie's linear and angular velocities to Webots linear and angular velocities
        v = v*self.sim_to_real_scale
        w = w

        # Calculate the wheel linear velocities
        vl = (v - self.wheel_radius/2*w) 
        vr = (v + self.wheel_radius/2*w)

        # Cap the angular velocities to the maximum speed
        omega_max = 6.279
        omega_l = vl/self.wheel_radius
        omega_r = vr/self.wheel_radius
        omega_current_max = max(abs(omega_l), abs(omega_r))
        if omega_current_max > omega_max:
            scale = omega_max/omega_current_max
            vl = vl*scale
            vr = vr*scale

            # Re-calculate the wheel angular velocities
            omega_l = vl/self.wheel_radius
            omega_r = vr/self.wheel_radius

        # Set the velocity of the motors. The left and right motor speeds need to be flipped.
        self.leftMotor.setVelocity(omega_r)
        self.rightMotor.setVelocity(omega_l)
        # print("omega_l, omega_r: ", omega_l, omega_r)

        # Run the simulation for a time step
        self.robot.step(self.TIME_STEP)
        # time.sleep(0.0001)


if __name__ == "__main__":
    
    # Create the Robot instance.
    robot = Robot()

    # Get Robot Name
    robot_name = robot.getName()

    global stopevent
    
    folder_path = os.path.dirname(os.getcwd())
    
    # Set the map path you want to test
    maze_name = "maze1"
    map_filename = os.path.join(folder_path, "exploration_controller","maps",maze_name+".json")
    
    #  Set state 'exploration'
    state = 'exploration'

    if len(sys.argv) > 1:
        map_filename = sys.argv[1]

    # robot_pause_time = 0.0
	
    # grid = Grid(map_filename)

    # # initial robot transformation (X, Y, yaw in deg)
    # robot_init_pose = grid.start
    # robot_init_pose.append(0) # needs to be 3d
    
    # robbie = Robot_Sim(robot_init_pose[0], robot_init_pose[1], robot_init_pose[2]) # Running from webots
    webot_robot = MoveDisrupter(robot, None, TIME_STEP,MAX_SPEED)
    webot_robot.robot_name = robot_name
    counter = 0
    state = 0

    while robot.step(TIME_STEP) != -1:
        # if state == 0:
        #     next_coord = [-1.0, 1.2]
        # elif state == 1:
        #     next_coord = [1.0, 1.2]

        # if webot_robot.goal_flag == True:
        #     state = not state
        #     webot_robot.goal_flag = False
        # if counter%10==0:
        #     webot_robot.move_to_next_coord(next_coord)
        # counter+=1
        v, w = webot_robot.receive_veloticy_command()
        webot_robot.move_motors(v, w)

    
    # robot_env = RobotEnvController(robbie, grid, webot_robot)
    # robot_env.program_state = state
    
    # stopevent = threading.Event()
    # gui = GUIWindow(grid,robot_env.program_state, stopevent)
    # robot_thread = RobotEnvThreadController(robot_env, gui, testing=False)
    # robot_thread.start()
    # gui.start()