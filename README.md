# ENPM661 Project 03: A* Path Planning for a Mobile Robot (Phase 1 & 2)

### For FalconSim part 2
The Duality Vibe Sim server (https://umd.vibesim.duality-robotics.io/) was returning a persistent "503 Service Temporarily Unavailable - nginx" error. Because the cloud environment was completely offline, I was not able to record the final video inside FalconSim. 

**Fallback Execution:** I have instead recorded the EXACT required Part 02 scenario running perfectly in Gazebo environment to prove the ROS2 node and A* algorithm are fully functional.

## Team Members
* Anusha Sharma (asharm50)

## Links
* **Part 01 Video:** https://drive.google.com/file/d/1vvpd1W1k08tzOd_taQYlb4nTkKjX-tx3/view?usp=sharing
* **Part 02 Video (Gazebo Fallback):** [Insert YouTube/Drive Link Here]

## Overview
This project implements the Backward A* search algorithm for a rigid mobile robot (Phase 1) and translates the algorithm into a ROS2 node for a Differential Drive TurtleBot3 (Phase 2). 

Unlike standard grid searches, Phase 1 navigates a continuous 600x250 workspace using a custom action set defined by a step size (L) and five steering angles. The robot is modeled with a 5mm radius and a 5mm clearance (10mm total bloat) around obstacles. Phase 2 applies the A* logic to a ROS2 workspace to navigate a simulated TurtleBot3 through a physical map using `cmd_vel` Twist messages.

## Dependencies
The following libraries are required to run this project:
* Python 3.x
* ROS2 (Humble recommended)
* `rclpy` (ROS2 Python Client Library)
* OpenCV (`cv2`) - Used for map generation and vector animation.
* NumPy (`numpy`) - Used for the 3D visited matrix and image generation.
* `math` and `heapq` (Standard Python libraries)

You can install the external python dependencies using pip:
```
pip install opencv-python numpy
```

## How to run:
Here are the step-by-step instructions to run the project:

1. Open your terminal and create a new workspace folder:
```
mkdir -p ~/proj3_ws/src
cd ~/proj3_ws/src
```
2. Clone this repository into src folder:
```
git clone [Insert Your GitHub Link Here] .
cd Part01
```
3. Run the main solver to generate the map and waypoints:
```
python3 main.py
```

    Part 01 sample input:
    ```
    Enter start coordinates:
    X coordinate (0-400): 0
    Y coordinate (0-200): 100
    Theta in degrees: 0

    Enter goal coordinates:
    X coordinate (0-400): 350
    Y coordinate (0-200): 100

    Enter RPM 1: 60
    Enter RPM 2: 120
    Enter Clearance in cm: 7    
    ```

4. Build the ROS2 packages:
```
cd ~/proj3_ws
colcon build --symlink-install
source install/setup.bash
```
5. Launch the simulation world:
```
ros2 launch turtlebot3_project3 competition_world.launch.py
```
6. Open another terminal tab and run controller node:
```
source install/setup.bash
ros2 run turtlebot_planner open_controller_loop
```
