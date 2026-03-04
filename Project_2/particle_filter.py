# Juan Jose Rosales Hernando

from grid import *
from particle import Particle
from utils import *
import setting
import numpy as np
np.random.seed(setting.RANDOM_SEED)
from itertools import product
from typing import List, Tuple


def create_random(count: int, grid: CozGrid) -> List[Particle]:
    """
    Returns a list of <count> random Particles in free space.

    Parameters:
        count: int, the number of random particles to create
        grid: a Grid, passed in to motion_update/measurement_update
            see grid.py for definition

    Returns:
        List of Particles with random coordinates in the grid's free space.
    """
    # TODO: implement here
    # -------------------
    particles = []
    for i in range(0, count):
        x, y = grid.random_free_place()
        h = random.uniform(0, 360)
        particles.append(Particle(x, y, h))
    return particles
    # -------------------
    

# ------------------------------------------------------------------------
def motion_update(old_particles:  List[Particle], odometry_measurement: Tuple, grid: CozGrid) -> List[Particle]:
    """
    Implements the motion update step in a particle filter. 
    Refer to setting.py and utils.py for required functions and noise parameters
    For more details, please read "Motion update details" section and Figure 3 in "CS3630_Project2_Spring_2025.pdf"


    NOTE: the GUI will crash if you have not implemented this method yet. To get around this, try setting new_particles = old_particles.

    Arguments:
        old_particles: List 
            list of Particles representing the belief before motion update p(x_{t-1} | u_{t-1}) in *global coordinate frame*
        odometry_measurement: Tuple
            noisy estimate of how the robot has moved since last step, (dx, dy, dh) in *local robot coordinate frame*

    Returns: 
        a list of NEW particles representing belief after motion update \tilde{p}(x_{t} | u_{t})
    """
    new_particles = []

    for particle in old_particles:
        # extract the x/y/heading from the particle
        x_g, y_g, h_g = particle.xyh
        # and the change in x/y/heading from the odometry measurement
        dx_r, dy_r, dh_r = odometry_measurement

        # TODO: implement here
        # ----------------------------------
        # align odometry_measurement's robot frame coords with particle's global frame coords (heading already aligned)
        dx, dy = rotate_point(dx_r, dy_r, h_g)

        # compute estimated new coordinate, using current pose and odometry measurements. Make sure to add noise to simulate the uncertainty in the robot's movement.
        dx_g_noisy = add_gaussian_noise(dx, setting.ODOM_TRANS_SIGMA)
        dy_g_noisy = add_gaussian_noise(dy, setting.ODOM_TRANS_SIGMA)
        dh_g_noisy = add_gaussian_noise(dh_r, setting.ODOM_HEAD_SIGMA)

        # **Step 3: Compute New Position**
        x = x_g + dx_g_noisy
        y = y_g + dy_g_noisy
        h = (h_g + dh_g_noisy) % 360  # Ensure heading remains within [0, 360)

        # create a new particle with this noisy coordinate
        new_particle = Particle(x, y, h)

        # ----------------------------------
        new_particles.append(new_particle)

    return new_particles

# ------------------------------------------------------------------------
def generate_marker_pairs(robot_marker_list: List[Tuple], particle_marker_list: List[Tuple]) -> List[Tuple]:
    """ Pair markers in order of closest distance

        Arguments:
        robot_marker_list -- List of markers observed by the robot: [(x1, y1, h1), (x2, y2, h2), ...]
        particle_marker_list -- List of markers observed by the particle: [(x1, y1, h1), (x2, y2, h2), ...]

        Returns: List[Tuple] of paired robot and particle markers: [((xp1, yp1, hp1), (xr1, yr1, hr1)), ((xp2, yp2, hp2), (xr2, yr2, hr2),), ...]
    """
    marker_pairs = []
    while len(robot_marker_list) > 0 and len(particle_marker_list) > 0:
        # TODO: implement here
        # ----------------------------------
        # find the (particle marker,robot marker) pair with shortest grid distance
        distance = 1000
        pos_closest_particle = -1
        pos_closest_robot = -1
        for i in range(len(robot_marker_list)):
            for j in range(len(particle_marker_list)):
                new_distance = grid_distance(robot_marker_list[i][0],
                                            particle_marker_list[j][0],
                                            robot_marker_list[i][1],
                                            particle_marker_list[j][1])
                if distance > new_distance:
                    distance = new_distance
                    pos_closest_robot = i
                    pos_closest_particle = j

        # add this pair to marker_pairs and remove markers from corresponding lists
        if pos_closest_robot != -1 and pos_closest_particle != -1:
            marker_pairs.append((particle_marker_list[pos_closest_particle],
                                 robot_marker_list[pos_closest_robot]))
            del particle_marker_list[pos_closest_particle]
            del robot_marker_list[pos_closest_robot]

        # ----------------------------------
    return marker_pairs

# ------------------------------------------------------------------------
def marker_likelihood(robot_marker: Tuple, particle_marker: Tuple) -> float:
    """ Calculate likelihood of reading this marker using Gaussian PDF. 
        The standard deviation of the marker translation and heading distributions 
        can be found in setting.py
        
        Some functions in utils.py might be useful in this section

        Arguments:
        robot_marker -- Tuple (x,y,theta) of robot marker pose
        particle_marker -- Tuple (x,y,theta) of particle marker pose

        Returns: float probability
    """
    l = 0.0
    # TODO: implement here
    # ----------------------------------
    # find the distance between the particle marker and robot marker
    distance_between_markers = grid_distance(robot_marker[0], robot_marker[1],
                           particle_marker[0], particle_marker[1])
    # find the difference in heading between the particle marker and robot
    # marker
    angle_between_markers = diff_heading_deg(particle_marker[2],
                                             robot_marker[2])

    # calculate the likelihood of this marker using the gaussian pdf. You can
    # use the formula on Page 5 of "CS3630_Project2_Spring_2025.pdf"
    l = np.exp(-((distance_between_markers**2)/(2*setting.MARKER_TRANS_SIGMA**2)
                 +((angle_between_markers**2)/(
                    2*setting.MARKER_HEAD_SIGMA**2))))

    # ----------------------------------
    return l

# ------------------------------------------------------------------------
def particle_likelihood(robot_marker_list: List[Tuple], particle_marker_list: List[Tuple]) -> float:
    """ Calculate likelihood of the particle pose being the robot's pose

        Arguments:
        robot_marker_list -- List of markers (x,y,theta) observed by the robot
        particle_marker_list -- List of markers (x,y,theta) observed by the particle

        Returns: float probability
    """
    l = 1.0
    marker_pairs = generate_marker_pairs(robot_marker_list, particle_marker_list)
    # TODO: implement here
    # ----------------------------------
    # update the particle likelihood using the likelihood of each marker pair
    # HINT: consider what the likelihood should be if there are no pairs generated

    # If no pairs are generated, return a default likelihood
    if not marker_pairs:
        return 0

    for i in range(0, len(marker_pairs)):
        l = l * marker_likelihood(marker_pairs[i][0], marker_pairs[i][1])

    # ----------------------------------
    return l

# ------------------------------------------------------------------------
def measurement_update(particles: List[Particle], measured_marker_list: List[Tuple], grid: CozGrid) -> List[Particle]:
    """ Particle filter measurement update
       
        NOTE: the GUI will crash if you have not implemented this method yet. To get around this, try setting measured_particles = particles.
        
        Arguments:
        particles -- input list of particle represents belief \tilde{p}(x_{t} | u_{t})
                before measurement update (but after motion update)

        measured_marker_list -- robot detected marker list, each marker has format:
                measured_marker_list[i] = (rx, ry, rh)
                rx -- marker's relative X coordinate in robot's frame
                ry -- marker's relative Y coordinate in robot's frame
                rh -- marker's relative heading in robot's frame, in degree

                * Note that the robot can only see markers which is in its camera field of view,
                which is defined by ROBOT_CAMERA_FOV_DEG in setting.py
				* Note that the robot can see mutliple markers at once, and may not see any one

        grid -- grid world map, which contains the marker information,
                see grid.py and CozGrid for definition
                Can be used to evaluate particles

        Returns: the list of particles represents belief p(x_{t} | u_{t})
                after measurement update
    """
    measured_particles = []
    particle_weights = []
    num_rand_particles = 25
    
    if len(measured_marker_list) > 0:
        for p in particles:
            x, y = p.xy
            if grid.is_in(x, y) and grid.is_free(x, y):
                robot_marker_list = measured_marker_list.copy()
                particle_marker_list = p.read_markers(grid)
                l = None

                # TODO: implement here
                # ----------------------------------
                # compute the likelihood of the particle pose being the robot's
                # pose when the particle is in a free space
                l = particle_likelihood(robot_marker_list, particle_marker_list)

                # ----------------------------------
            else:
                # TODO: implement here
                # ----------------------------------
                # compute the likelihood of the particle pose being the robot's pose
                # when the particle is NOT in a free space
                l = 0

                # ----------------------------------

            particle_weights.append(l)
    else:
        particle_weights = [1.]*len(particles)
    
    # TODO: Importance Resampling
    # ----------------------------------
    # if the particle weights are all 0, generate a new list of random particles
    if all(w == 0 for w in particle_weights):
        return create_random(setting.PARTICLE_COUNT, grid)

    # normalize the particle weights
    sum_of_weights = sum(particle_weights)
    for i in range(0, len(particles) - 1):
        particle_weights[i] = particle_weights[i] / sum_of_weights


    # create a fixed number (num_rand_particles) of random particles and add to measured particles
    measured_particles.extend(create_random(num_rand_particles, grid))

    # resample remaining particles using the computed particle weights, make sure there are setting.PARTICLE_COUNT particles in measured_particles list
    measured_particles.extend(random.choices(particles,
                                             particle_weights,
                                             k=setting.PARTICLE_COUNT))


    # ----------------------------------

    return measured_particles


