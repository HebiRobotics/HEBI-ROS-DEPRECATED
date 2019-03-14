# Overview

These files are used to interface with the omnibase robot base. This documentation provides instruciton to run each of the nodes, as well as information on the ROS messages used by each file.

**Important: to run any of these commands, you will need to run `source devel/setup.sh` from your catkin workspace first**

# Running the code

## Omnibase nodes:

To run the omnibase odometry node using the keyboard teleop code, use this command:
```
roslaunch hebiros_advanced_examples example_omnibase_odometry.launch 
```

To run the omnibase odometry node using the Mobile I/O app, use this command:
```
roslaunch hebiros_advanced_examples omnibase_navigation_mobile_IO.launch 
```

To run the omnibase free_roam node, use this command:
```
roslaunch hebiros_advanced_examples example_omnibase_free_roam.launch
```

## Mobile I/O app interface:

To run just the Mobile I/O app interface, run the following command:
```
python mobile_io_controller.py
```
*Note: In order to run this command, there must be an existing rosmaster node running on the system*


# Rostopic details for each file:

## omnibase_free_roam

The free_roam node subscribes to the `/cmd_vel` rostopic; the node converts the velocity messages from the topic to movement commands for the omnibase robot.


## omnibase_odometry

The odometry node subsrcibes to the `/joint_state` rostopic; on this topic, the node recieves position data for each of the omnibase actuators.

The odometry node publishes to the `/odom` rostopic; the node pushes odometery data to this topic at a rate of 10Hz.


## mobile_IO_controller

The mobile_io_controller python script uses the hebi.Lookup() command to connect to a valid Mobile I/O app on the network.
It receives commands from and sends feedback to the app.

The mobile_io_controller python script publishes to the '/cmd_vel' rostopic;
Velocity commands for the omnibase node are sent over this rostopic.

