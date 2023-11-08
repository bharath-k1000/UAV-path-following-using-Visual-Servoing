# UAV-path-following-using-Visual-Servoing
Visual Servoing based autonomous UAV path following implemented with ROS1 Noetic, PX4, mavros, Pygame and Gazebo.

Current implementation uses thresholding for path segmentation but any image segmentation can be implemented easily instead. Further work aims to use Meta's SAM to get much better path segmentation results.

Implemented trajectory controller for UAV using keyboard input using mavros and Pygame. 

Centroid of the area of segmented path is calculated and using PID controller and matrix transformations, robot's pose and orientation are updated to align with path.

TO RUN AND SIMULATE PATH-FOLLOWING IN GAZEBO

1. Launch PX4
Preferaby in baylands.world

2. Launch mavros

3. python3 path_follower.py

Control drone using key mapping specified in code file. 

Take off with 'L' and land with 'O'.

Activate Visual servoing mode with 'V'
