# Overview

These files are used to interface with the omnibase robot base. This documentation provides instruction to run each of the nodes, as well as information on the ROS messages used by each file.

**Important: to run any of these commands, you will need to run `source devel/setup.sh` from your catkin workspace first**


# Initial Setup

First, we recommend using the packaged installation of the HEBI C++ API; replace `<distro>` with the distribution you are using.

```
sudo apt install ros-<distro>-hebi_cpp_api
```

In a new directory (or your existing catkin workspace), run the following:
```
mkdir src
cd src
git clone https://github.com/HebiRobotics/HEBI-ROS.git
cd HEBI-ROS
git checkout develop
cd ..
catkin_init_workspace
cd ..
```
To build the catkin workspace, run the following commands:
```
catkin_make clean
catkin_make
source devel/setup.bash
```

# Running the code

## Omnibase nodes:

The omnibase free_roam node creates the HEBI-ROS node, and connects to all three of the HEBI modules on the omnibase. This node listens to commands sent on `cmd_vel` and converts velocity commands into movement commands for the HEBI modules. To launch use this command:
```
roslaunch hebiros_advanced_examples example_omnibase_free_roam.launch
```

The omnibase odometry node is used to publish the odometry data of the omnibase to the `/odom` topic. This is necessary in order to use any mapping or navigation packages with the omnibase node.
This launch file also runs the `key_read` python file, which will allow the user to send velocity commands over `cmd_vel` using keyboard input. A Pygame instance will be created and will display instructions for keyboard input commands. Use this command to run this node: 
```
roslaunch hebiros_advanced_examples example_omnibase_odometry.launch 
```

This launch file is the same as example_omnibase_odometry, but instead of running the `key_read.py`file, this launch file runs the `mobile_IO_controller.py` file, which will allow the user to send command velocities to the omnibase using a Mobile I/O controller. To launch, use this command:
```
roslaunch hebiros_advanced_examples omnibase_teleop_mobile_IO.launch 
```
**Note: If you use the Mobile I/O controller, be sure to disable the controller (press B8) before closing the ROS master node on your PC**

## Mobile I/O app interface:

To run just the Mobile I/O app interface, run the following command:
```
python mobile_IO_controller.py
```
*Note: In order to run this command, there must be an existing rosmaster node running on the system*


# Rostopic details for each file:

## omnibase_free_roam

The free_roam node subscribes to the `/cmd_vel` rostopic; the node converts the velocity messages from the topic to movement commands for the omnibase robot.


## omnibase_odometry

The odometry node subscribes to the `/joint_state` rostopic; on this topic, the node receives position commands for each of the omnibase actuators.

The odometry node publishes to the `/odom` rostopic; the node pushes odometery data to this topic at a rate of 10Hz.


## mobile_IO_controller

The mobile_IO_controller python script uses the hebi.Lookup() command to connect to a valid Mobile I/O app on the network.
It receives commands from and sends feedback to the app.
For more information, please go to this link: http://docs.hebi.us/tools.html#mobile-io

The mobile_IO_controller python script publishes to the '/cmd_vel' rostopic;
Velocity commands for the omnibase node are sent over this rostopic.

*Note: the device running the Mobile I/O app must be connected on the same network* 

