# ROSie demo

This code was run at IROS 2018 in Madrid, and is designed to be run with a HEBI Rosie robot kit, including a Intel Realsense camera mounted on a vertical pole and a basket to catch bean bags that are picked up by the robot.

# Compiling

After installing the necessary Intel Realsense drivers and ROS packages, compile via `catkin_make` in your ROS workspace.

# Running the demo

The demo is controlled via the HEBI Mobile I/O app.  Several buttons control the main modes of the demo:

* B1 - pause
* B2 - continue autonomous mode
* B3 - calibrate
* B5 - quit

# Camera to world calibration

Note that there are four .svg files in their directory.  These should be printed out and taped together with the red dots in the center.  Be careful that these printouts are full scale on 8 1/2 x 11" paper.

Once the demo is paused, the complete 4-page calibration sheet should be centered on the floor at the front of the robot, with the blank space adjacent to the robot.

(TODO: photo of calibration setup)

Then B3 can be pressed, at which time the screen should show the captured image of the calibration sheet, including the recognized circles.

# Color calibration

You may need to tune the RGB or HSV values in order to pick up colored objects in the particular lighting conditions you run this demo in.  To do so, we have included a ros program that allows you to drag sliders to adjust these values.  Note that you currently need to change the source to adjust to GUI between RGB and HSV values.

Once you have ranges for each of these parameters, you can add set these in the `vision_process.cpp` file as the various "color" objects that you are looking for.

# Code structure

The `demo_central.cpp` file/node provides the high-level logic for the demo (e.g., calling the vision service, telling the base to move, and telling the arm to move).

The `Rosie_arm_node.cpp` file/node provides a high-level interface for the arm hardware (e.g., taking commands for the arm to move to a cartesian point, and sending these as smooth trajectories to the hardware).  This consumes the `arm.hpp/cpp`, `arm_kinematics.hpp/cpp`, and `arm_trajectory.hpp/cpp` files/classes.

The `Rosie_base_node.cpp` file/node provides a high-level interface for the base hardware (e.g., taking commands for the base to move a certain distance, and sending these as smooth trajectories to the hardware)

The `vision_process.cpp` file/node provides basic image processing using the Realsense's RGB camera, and returns the position of identified colored blobs in the image.

The `gripper_node.cpp` file/node is the hardware interface for the robot's gripper.

The `vision_threshold.cpp` file is a separate node that provides a GUI for testing different threshold values in real time.


