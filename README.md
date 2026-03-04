# Webots Robotics Projects

A series of 6 progressive robotics projects implemented in [Webots](https://cyberbotics.com/), exploring core concepts in robot perception, localization, planning, and autonomous exploration. All projects use the **e-puck** robot platform.

> **Note:** Projects 1 and 2 use underscored folder names (`Project_1/`, `Project_2/`); Projects 3–6 use space-separated folder names (`Project 3/` … `Project 6/`).

---

## Project 1 – Vision & LiDAR Object Detection

**Folder:** `Project_1/`

### Overview
The robot detects a randomly spawned object (a sphere or a disk) using its onboard camera and LiDAR sensor, estimates the object's distance, and classifies its shape.

A **Supervisor** controller randomizes each trial by spawning either a sphere or a disk at a random position within the robot's field of view. Two detection approaches are compared:

| Controller | Sensors Used | Output |
|---|---|---|
| `proj1_vision_only_controller` | Camera only | Distance (depth) and angle |
| `proj1_vision_lidar_controller` | Camera + LiDAR | Distance (LiDAR) and shape classification |

### Key Concepts
- **Camera depth estimation** – using the known object diameter and the pinhole camera model (focal length + apparent size) to estimate depth.
- **LiDAR distance measurement** – mapping the detected pixel angle to the matching LiDAR ray and interpolating between adjacent measurements for higher accuracy.
- **Shape classification** – comparing neighbouring LiDAR readings around the detected angle: a disk presents a flat LiDAR profile while a sphere shows asymmetry.
- **Circle detection** with OpenCV (Hough Circle Transform).

### Dependencies
```
opencv-python==4.9.0.80
matplotlib==3.7.4
```

---

## Project 2 – Particle Filter Localization (2D Grid)

**Folder:** `Project_2/`

### Overview
A classic **particle filter** (Monte Carlo Localization) implemented on a 2-D occupancy grid. The robot (simulated offline) estimates its pose by maintaining a set of weighted particles that represent the probability distribution over possible positions and headings.

### Key Concepts
- **Motion update** – each particle is propagated using odometry measurements expressed in the robot frame, then rotated into the world frame, and Gaussian noise is added to model motion uncertainty.
- **Measurement update** – particle weights are computed by comparing the robot's observed markers (distance + heading) against the markers visible from each particle's hypothetical pose, using a Gaussian likelihood model.
- **Importance resampling** – particles are resampled proportionally to their weights; a fixed number of uniformly random particles are injected each step to avoid particle depletion.
- **Marker pairing** – observed markers are greedily matched to expected markers by minimum grid distance before computing the joint likelihood.

### Key Files
| File | Description |
|---|---|
| `particle_filter.py` | Core algorithm: `motion_update`, `measurement_update`, `marker_likelihood` |
| `particle.py` | `Particle` class (x, y, heading + marker visibility) |
| `grid.py` | `CozGrid` occupancy grid |
| `pf_gui.py` / `gui.py` | Tkinter-based visualisation |
| `setting.py` | Noise parameters and random seed |

### Dependencies
```
numpy
tk
```

---

## Project 3 – LiDAR Particle Filter in a Webots Maze

**Folder:** `Project 3/`

### Overview
A Webots simulation where an e-puck robot navigates a maze and localizes itself using a **LiDAR-based particle filter**. Unlike Project 2 (marker-based), this project uses raw LiDAR range arrays compared against a simulated LiDAR model built from the maze's wall geometry.

### Robot Trajectory
The robot executes a fixed sequence of moves (forward → turn right → forward → turn right → forward) while running the particle filter in parallel on every 5th timestep.

### Key Concepts
- **LiDAR simulation** (`lidar_sim.py`) – ray-casting against rectangular wall obstacles to simulate expected LiDAR readings at any hypothetical pose.
- **Particle likelihood** – Gaussian comparison between real and simulated LiDAR arrays.
- **Differential-drive odometry** – wheel encoder deltas are converted to a relative SE(2) transform used in the motion update.
- **Data capture mode** – the controller can record LiDAR, pose, and odometry data to CSV files for offline analysis.
- **Confidence check** – the estimated pose is scored against ground truth (GPS + compass) to compute accuracy.

### Key Files
| File | Description |
|---|---|
| `particle_filter/particle_filter.py` | LiDAR-based PF: `motion_update`, `measurement_update` |
| `particle_filter/lidar_sim.py` | Ray-casting LiDAR simulator |
| `particle_filter/environment.py` | Loads world config (walls, robot params) |
| `controllers/proj3_maze_world1_controller/` | Webots robot controller |
| `worlds/maze_world1.wbt` | Webots world file |

### Dependencies
```
numpy
tk
```

---

## Project 4 – Bayesian State Estimation (Markov Localization)

**Folder:** `Project 4/`

### Overview
The robot moves between named rooms (e.g., Living Room, Kitchen, Office, Hallway, Dining Room) and maintains a **Bayesian belief** over its current location using a conditional probability table (CPT) that encodes transition probabilities for each action.

### Key Concepts
- **State estimation** – the robot's state is its current room; `compute_state` samples the next room from the CPT given the action taken.
- **Belief update** – `compute_belief` performs a matrix-vector multiplication between the current belief vector and the transition probability matrix for the executed action, then normalizes the result.
- **Maximum a posteriori estimate** – the room with the highest belief probability is used as the robot's best position estimate.
- **Multiple maps** – the system supports different building layouts (`map_house`, `map_office`) defined in `config.py`.

### Key Files
| File | Description |
|---|---|
| `robot.py` | `Robot` class: `compute_state`, `compute_belief`, `update` |
| `config.py` | Map definitions with CPTs and room coordinates |
| `gui.py` / `pa_gui.py` | Visualisation GUI |
| `setting.py` | Shared settings |

### Dependencies
```
numpy
tk
```

---

## Project 5 – RRT Path Planning in Webots

**Folder:** `Project 5/`

### Overview
The robot plans a collision-free path through a maze using the **Rapidly-exploring Random Tree (RRT)** algorithm and then executes that path in Webots by navigating node-by-node.

### Key Concepts
- **RRT** – iteratively samples random nodes, finds the nearest tree node, extends the tree towards the sample, and checks for goal reaching.
- **Path execution** – the robot turns in place to face each successive waypoint (`turn_in_place`) and then drives forward to it (`move_forward`), using GPS and compass for closed-loop control.
- **Map representation** – the maze is encoded as a JSON file; obstacle checks and goal detection are handled by the `Map` class.

### Key Files
| File | Description |
|---|---|
| `controllers/rrt_controller/rrt.py` | `RRT` and `RRT_visualize` algorithms |
| `controllers/rrt_controller/rrt_controller.py` | Webots robot controller + `MoveRobot` class |
| `controllers/rrt_controller/map.py` | Map loading, node management, collision checking |
| `controllers/rrt_controller/maps/` | JSON maze definitions |

---

## Project 6 – Autonomous Exploration with Disrupters

**Folder:** `Project 6/`

### Overview
The robot autonomously explores an unknown grid-based environment to find all hidden markers, while avoiding **disrupter robots** that try to obstruct it. The exploration strategy is a frontier-based approach driven by a state machine.

### Key Concepts
- **Frontier-based exploration** – the robot identifies the boundary between explored and unexplored cells (frontiers) and navigates to the nearest frontier centroid.
- **State machine** (`exploration_state_machine`) – the robot cycles through states such as computing a new goal, planning a path (RRT), and executing the path.
- **Disrupter robots** – adversarial robots that follow their own trajectories; the exploration controller broadcasts velocity commands to them via Webots emitter/receiver.
- **PID controller** – smooth motion to waypoints using separate linear and angular PID loops.
- **Ground-truth tracking** – robot position is obtained from a Supervisor via the Webots receiver channel, not from GPS alone.
- **Grid mapping** – an occupancy grid is updated as the robot moves, enabling frontier detection and collision-free planning.

### Key Files
| File | Description |
|---|---|
| `controllers/exploration_controller/exploration.py` | `exploration_state_machine`, `get_wheel_velocities` |
| `controllers/exploration_controller/exploration_controller.py` | Webots controller + `RobotEnvController` + `MoveRobot` |
| `controllers/exploration_controller/robot.py` | `Robot_Sim` and `PidController` |
| `controllers/exploration_controller/grid.py` | Occupancy grid + frontier detection |
| `controllers/exploration_controller/disrupter.py` | Disrupter robot update logic |
| `controllers/exploration_controller/maps/` | JSON maze definitions |

---

## Project Summary

| # | Project | Main Algorithm | Sensors |
|---|---|---|---|
| 1 | Vision & LiDAR Object Detection | Pinhole camera model + LiDAR interpolation | Camera, LiDAR |
| 2 | Particle Filter (grid) | Monte Carlo Localization | Simulated markers |
| 3 | LiDAR Particle Filter (Webots maze) | LiDAR-based MCL + ray-casting | LiDAR, encoders, GPS, compass |
| 4 | Bayesian State Estimation | Markov localization (CPT belief update) | Abstract actions |
| 5 | RRT Path Planning | Rapidly-exploring Random Tree | GPS, compass, encoders |
| 6 | Autonomous Exploration | Frontier exploration + RRT + PID | LiDAR, GPS, compass, emitter/receiver |

---

## General Requirements

- [Webots](https://cyberbotics.com/) R2023b or later
- Python 3.10+
- See each project's `requirements.txt` for Python package dependencies
