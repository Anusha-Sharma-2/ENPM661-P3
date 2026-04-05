# ENPM661-P3# ENPM661 Project 03 Phase 1: Backward A* Path Planning for a Mobile Robot

Implementation of continuous path planning for a rigid mobile robot using the Backward A* search algorithm in a 3D configuration space (X, Y, Theta).

**Team Members:**
* Anusha Sharma (Directory ID: asharm50, UID: 119789247)

## Overview
This project implements the Backward A* search algorithm to find the optimal path for a rigid mobile robot. Unlike standard grid searches, this algorithm navigates a continuous 600x250 workspace using a custom action set defined by a step size (L) and five steering angles (-60, -30, 0, 30, 60). The robot is modeled with a 5mm radius and a 5mm clearance, resulting in a 10mm total bloat around all boundaries and semi-algebraic obstacles. Duplicate nodes are detected using a discrete 3D visited matrix (0.5 unit threshold for coordinates, 30° threshold for orientation).

## Dependencies
The following libraries are required to run this project:
* Python 3.x
* OpenCV (`cv2`) - Used for map generation and vector animation.
* NumPy (`numpy`) - Used for the 3D visited matrix and image generation.

You can install the necessary external dependencies using pip:

    pip install opencv-python numpy

*(Note: `math` and `heapq` are also heavily utilized for trigonometric calculations and the priority queue, but are part of the standard Python library and do not require installation).*

## Run Instructions
Ensure that `map.py` and the main solver script are located in the same directory.

1. Open your terminal or command prompt and navigate to the project directory.
2. To run the **Backward A*** algorithm, execute the following command:
    
    `python a_star_anusha_sharma.py`
    
3. Upon running the script, the terminal will prompt you to enter the Start coordinates, Goal coordinates, and Step Size. 
    * **X inputs:** Must be between 0 and 600.
    * **Y inputs:** Must be between 0 and 250.
    * **Theta (Orientation):** Must be entered in degrees as a multiple of 30 (e.g., 0, 30, 60, -30).
    * **Step Size (L):** Must be an integer between 1 and 10.
    * **Note:** If you enter coordinates that lie within a bloated obstacle or the 10mm clearance boundary, the program will notify you and ask for new coordinates until valid ones are provided.

4. Once a valid configuration is entered, the algorithm will begin the search. Upon reaching the goal threshold (1.5 units), it will backtrack to find the optimal path.
5. The program will automatically generate an OpenCV animation of the search vectors and the final path. Once rendering is complete, the video will be saved in the directory as `animation.mp4`.

Full example, for ease:
```
Enter start coordinates:
X coordinate (0-600): 15
Y coordinate (0-250): 15
Theta (degrees, multiple of 30): 30

Enter goal coordinates:
X coordinate (0-600): 580
Y coordinate (0-250): 230
Theta (degrees, multiple of 30): 60

Enter Step Size L (1 to 10): 10
Path generated with 76 steps!
Starting animation generation:
Animation saved as animation.mp4!
```