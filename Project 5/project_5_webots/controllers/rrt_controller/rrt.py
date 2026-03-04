import sys
import time
from map import Map
from gui import *
from utils import *
from robot_sim import *
import os

MAX_NODES = 20000
final_path = []

def RRT(map):
    """ 
    Apply the RRT algorithm to path through this
    map.
    """
    global final_path 

    map.add_node(map.get_start())
    map_width, map_height = map.get_size()
    while (map.get_num_nodes() < MAX_NODES):
        # TODO: your code here
        ########################################################################

        node_rand = map.node_generator()
        distance = map_width + map_height # initialize with large value
        node_near = map.get_start()
        node_new = node_rand
        line_segment = (node_near, node_new)


        for node in map.get_nodes():
            d = get_dist(node, node_rand)
            if d <= distance:
                node_near = node
                distance = d
                
        node_new = map.step_from_to(node_near, node_rand)
        line_segment = (node_near, node_new)

      
        map.add_path(node_near, node_new)
        
        

        ########################################################################
        time.sleep(0.01)
        if map.is_solved():
            break
    print('Out of while')
    path = map.get_path()
    print('path done')
    smoothed_path = map.get_smooth_path(10)
    print('path smooth')
    if map.is_solution_valid():
        print("A valid solution has been found :-) ")
        print("Nodes created: ", map.get_num_nodes())
        print("Path length: ", len(path))
        print("Smoothed path length: ", len(smoothed_path))
        
        # Store robot path
        robot_path = []
        for item in smoothed_path:
            robot_path.append((item.x,item.y))
        final_path = robot_path
    else:
        print("Please try again :-(")


class RobotThread(threading.Thread):
    """Thread to run vector code separate from main thread
    """

    def __init__(self, robot, map ):
        threading.Thread.__init__(self, daemon=True)
        self.robot = robot
        self.map = map

    def run(self):
        robot_planning_with_exploration(self.robot, self.map)
        time.sleep(5)
        stopevent.set()


class RRTThread(threading.Thread):
    """Thread to run RRT separate from main thread
    """

    def __init__(self, map):
        threading.Thread.__init__(self, daemon=True)
        self.map = map

    def run(self):
        self.path = RRT(self.map)
        time.sleep(5)
        self.map.reset_paths()
        stopevent.set()     


def RRT_visualize(map):
    
    global stopevent
    stopevent = threading.Event()
    exploration = False
    for i in range(0,len(sys.argv)): 
        #reads input whether we are running the exploration version or not
        if (sys.argv[i] == "-explore"):
            exploration = True
    
    if exploration:
        r = DDRobot(map.get_start().x, map.get_start().y, map)
        robot_thread = RobotThread(robot=r, map=map)
        visualizer = Visualizer(map, r, stopevent, exploration)
        robot_thread.start()
        visualizer.start()
    else:
        rrt_thread = RRTThread(map=map)
        visualizer = Visualizer(map, None, stopevent, exploration)
        rrt_thread.start()
        visualizer.start()

    return final_path

if __name__ == '__main__':
    map = Map("maps/maze1.json")
    RRT_visualize(map)