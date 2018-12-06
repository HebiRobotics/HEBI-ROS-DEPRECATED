# ROSie demo

This code was run at IROS 2018 in Madrid, and is designed to be run with a HEBI Rosie robot kit, including a Intel Realsense camera mounted on a vertical pole and a basket to catch bean bags that are picked up by the robot.

# Install dependencies 

See [Install Instructions](INSTALL.md)

# Compiling

After installing the necessary Intel Realsense drivers and ROS packages, compile via `catkin_make` in your ROS workspace (~/rosie_workspace if you have followed the installation instructions above).

# Configuring the robot

If you have not yet run the demo on this robot, or the control gains may have been changed since you last ran the program, open the Scope diagnostic program.  First, select the three wheel modules in the left pane (they should be in order, 1-3).  Then go to the "gains" tab, ensure the "persist sends" toggle is turned on, and then click the "send file" button.  Select the `rosie_wheel_gains.xml` file from the ROSie source directory, and click "ok" to configure these modules.

Next, select the remaining Rosie modules (base, elbow, shoulder, spool, wrist1, wrist2, and wrist3), and click the "send file" button again.  This time, select the `rosie_arm_gains.xml` file from the ROSie source directory, and click "ok" to configure these modules.

# Color calibration for object detection

**Color detection thresholds should be tuned for each object that is being recognized each time the lighting conditions change significantly**

You may need to tune the RGB or HSV values in order to pick up colored objects in the particular lighting conditions you run this demo in.  To do so, we have included a ROS node and launch file that allows you to drag sliders to adjust these values.  Note that you currently need to change the source to adjust to GUI to show the effect of different RGB value thresholds.

To run the GUI, run:
```roslaunch example_nodes VisionThreshold.launch```

Tune min/max rgb values to get clear segmentation for desired objects, with as little extra noise as possible.  The object to be picked up should be displayed as white, and the background should be black.  Note min/max for each channel.

Once you have ranges for each of these parameters, you can add set these in the `vision_process.cpp` file as the various "color" objects that you are looking for.  Change or add the color structure for your identified object here:

https://github.com/HebiRobotics/HEBI-ROS/blob/04748b1b3a712a4fb881e365af1127304d29c19e/example_nodes/src/Rosie_Demo/vision_process.cpp#L53

And then enable it in the logic that is actually searching for blobs here:

https://github.com/HebiRobotics/HEBI-ROS/blob/04748b1b3a712a4fb881e365af1127304d29c19e/example_nodes/src/Rosie_Demo/vision_process.cpp#L251

After making these changes, run `catkin_make` in your workspace to have these values used next time the code is run.

(Advanced note -- you can tweak the source of the vision threshold and vision process programs to allow HSV as well.)

# Starting the demo

Before running, ensure that the all modules in the arm (Base, shoulder, elbow, and wrists) are near 0 (especially the base module).  When you start the program, the robot will move into a "home" position with the end effector in front of the robot, and if the modules and a large motion could cause the robot arm to collide with the mast holding the realsense up.

To begin the program, launch ROSie.launch from the example_nodes package.  Note: you must be in the catkin workspace you set up for this to work (e.g., the `~/rosie_workspace` directory):

```roslaunch example_nodes ROSie.launch```

After starting the program, the robot should move to its base position.  At this time, ensure there is no red text on the screen (this indicates a problem with one of the nodes -- either the gripper, base, or arm modules cannot be found, or the computer cannot connect to the realsense).  If the modules cannot be found, <ctrl-C>, then check the connections between modules and try to restart the program.  If the realsense cannot be found, then <ctrl-C>, unplug and replug the realsense, and relaunch the program.
  
Once a successful launch occurs, the program will be waiting for a Mobile IO device to appear on the robot's network.  This can be an android or iPhone, with the HEBI Mobile IO app installed.  This will be used to control the robot.

If this cannot be found, ensure that the device running the Mobile IO app is connected the the `hebi_robot` wifi network, with a password of `hebi123411`.
- If not set correctly, set the app name to "Mobile IO" and family to "HEBI".  Test from Scope on the robot to ensure the app is visible from the robot's PC.  In the iOS app, you can set this from the app settings; in Android, this is settable via Scope.
- Ensure the app is connected to the `hebi_robot` robot!

After the Mobile IO app is found, *you must press (1) on the app to begin the rest of the program!*

# Running the demo

Several buttons control the main modes of the demo:

* B1 - initialize on boot, pause while running
* B2 - continue autonomous mode
* B3 - calibrate
* B4 - deploy bean bags (so they can be picked up again)
* B5 - quit

**Important: camera + vision calibration should be completed before attempting autonomous mode!!!**

Camera to robot calibration should be completed each time the camera changes position/angle relative to the robot, or is added/removed from the robot (E.g., for packing).  See below for instructions.

Color detection thresholds should be tuned for each object that is being recognized each time the lighting conditions change significantly.  The current classifier is a simple image thresholding + blob detection approach; more sophisticated methods could be added if desired.  See above for instructions.

## Basic autonomous functionality

When "B2" is pressed, the robot will transition to "autonomous mode".  When in this mode, the robot will scan the area immediately around it for colored beanbags.  If it sees one it can reach, the robot grabs it and places it in its basket.  If it cannot reach, it moves towards the object.  If it cannot find an object, it will continue searching.

When it has cleaned up the entire area around it, it will "redeploy" its beanbags by tossing them on the floor, then moving to a slightly different starting point.

When in autonomous mode, the LEDs on the system can be used for visial diagnostic of the autonomous algorithm.  If the arm is picking a yellow beanbag, the LEDs on the arm will turn yellow.  If the base is moving towards a red beanbag, the LEDs on the three wheel modules will turn red.

# Camera to robot calibration

Note that there are four .svg files in their directory.  These should be printed out and taped together with the red dots in the center.  Be careful that these printouts are full scale on 8 1/2 x 11" paper.

Once the demo is paused, the complete 4-page calibration sheet should be centered on the floor at the front of the robot, with the blank space adjacent to the robot.  Carefully center the calibration sheet in the middle of the robot, and ensure the close edge of the paper is flush with the vertical front panel of the robot:

![Side view of calibration](calibration_side.jpg)
![Front view of calibration](calibration_front.jpg)

Then B3 can be pressed, at which time the screen should show the captured image of the calibration sheet, including the recognized circles (these will show up as bright concentric circles -- there are also purple circles which may show up, but these are a diagnostic that just indicate all of the circular blobs were found in the image.

Note with some combinations of realsense drivers and openCV, you may need mutiple calibration attempts before one finally matches.  Also, the displayed image may not be shown on the first time.  However, the content in the terminal should clearly state whether or not the calibration succeeded.

_If the calibration fails_, press "B1" to "pause" the control again, then press B3 to calibrate again.

If the captured image does not show up, trying 3-4 more times will usually work to help visualize the image.

**Important: Note that this calibration only applies for the current run of the robot; to set this as the default starting camera calibration, save these values in the source code at the location below, and be sure to recompile by running `catkin_make` in your workspace to have these values used next time the code is run. **

https://github.com/HebiRobotics/HEBI-ROS/blob/04748b1b3a712a4fb881e365af1127304d29c19e/example_nodes/src/Rosie_Demo/demo_central.cpp#L327

# Code structure

The `demo_central.cpp` file/node provides the high-level logic for the demo (e.g., calling the vision service, telling the base to move, and telling the arm to move).

The `Rosie_arm_node.cpp` file/node provides a high-level interface for the arm hardware (e.g., taking commands for the arm to move to a cartesian point, and sending these as smooth trajectories to the hardware).  This consumes the `arm.hpp/cpp`, `arm_kinematics.hpp/cpp`, and `arm_trajectory.hpp/cpp` files/classes.

The `Rosie_base_node.cpp` file/node provides a high-level interface for the base hardware (e.g., taking commands for the base to move a certain distance, and sending these as smooth trajectories to the hardware)

The `vision_process.cpp` file/node provides basic image processing using the Realsense's RGB camera, and returns the position of identified colored blobs in the image.

The `gripper_node.cpp` file/node is the hardware interface for the robot's gripper.

The `vision_threshold.cpp` file is a separate node that provides a GUI for testing different threshold values in real time.


