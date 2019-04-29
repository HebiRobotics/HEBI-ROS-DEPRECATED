# Omnibase Navigation & Mapping Instructions


**Important: In order to run the Navigation & Mapping code, you must already have a catkin workspace setup with omnibase code running using the HEBI-ROS and the hebi-cpp-api-ros repositories. Please visit this link for setup instruction: https://github.com/HebiRobotics/HEBI-ROS/tree/develop/hebiros_advanced_examples/src/omnibase**


# Installation & Setup

Before you can start building maps and using the navigation stack with your robot, you need to add the following ROS repositories to your /SRC folder in your catkin workspace. These are the open source libraries used for mapping (GMapping) and navigation in ROS. They are currently compatible with ROS Kinetic and Melodic.

```
git clone https://github.com/ros-perception/openslam_gmapping
```
```
git clone https://github.com/ros-perception/slam_gmapping.git
```
```
git clone https://github.com/ros-planning/navigation.git
```
```
git clone https://github.com/ros/geometry2.git
```
```
git clone https://github.com/ros-planning/navigation_msgs.git
```


### FOR KINETIC ONLY -- FIX BUILD OF NAVIGATION PACKAGE!
```
cd navigation
git checkout kinetic-devel
rm -r fake_localization
cd ..
```
## RPLidar Setup

### USB Configuration

In order to use any USB-based Lidar, you need to enable access to the USB ports serial connection (/dev/ttyUSB0). The easiest way to do this is by adding your user account to the dialout group, using this command:
```
sudo gpasswd --add ${USER} dialout
```

### Filtering

The laser filtering is completed with the `laser_filters` packages (wiki.ros.org/laser_filters), which reads from topic `scan` and publishes filtered data to `scan_filtered`.

The two useful filters we have user are the "Angular Bounds" and "Angular Bounds In Place" filters. The Angular Bounds Filter has been created in order to ignore laser scans outside of a certain field of view. The bounds of the obstruction can be set with `lower_angle` and `upper_angle`. The Angular Bounds In Place Filter has been created in order to ignore laser scans within a certain subset of the field of view. The bounds of the obstruction can be set with `lower_angle` and `upper_angle`.

The parameters for a default setup (ignoring data in a 60 degree arc behind the robot) is found in `resources/params/laser_filters.yaml`.

To test and tune, you can run the "mapping" node below, and watch the laser returns; this filter is automatically applied (via the launch files) when running the mapping and navigation nodes below.

### Mounting Position / Configuration

The relative position of the LIDAR is stored in the `/urdf/omnibase.xacro` file; find the `scan_joint` joint near the bottom of the file, and set the x/y/z and r/p/y to match the mounting position (relative to the center of the base frame)


## Rosie Mapping

In order to run the mapping demo on the omnibase, there are two files that need to be run on omnibase's cpu.S

The first is the omnibase_navstack file, which is a modified version of the odometry.cpp file which outputs the correct odometry data. This can be run using this command:
```
roslaunch hebiros_advanced_examples omnibase_navigation_mobile_IO.launch 
```

The second file that needs to be run is the gmapping launch file. This starts up the RPLidar module as well as RViz. Use this command:
```
roslaunch hebiros_advanced_examples omnibase_gmapping.launch 
```

Using the Mobile I/O controller, drive the omnibase around to build a map. When the map is finished, use the map server to save the map to the local directory with the filename FILENAME:
```
rosrun map_server map_saver -f FILENAME
```

(Note: running the map_saver command should be done in the same directory which you are saving the map to; if you try to specify a directory or path in the "FILENAME", you will need to manually edit the .yaml file after saving to fix the relative path reference between yaml and pgm)


## Rosie Navigation

In order to run the navigation demo, a couple things need to be taken care of prior.
First, you must have created a map using the gmapping demo (.pgm and .yaml files); you must set this map in the omnibase_navigation.launch file as the default map.
Second, the robot must have the RPLidar attached and needs to start near the origin of the map.

Two files need to be run in order to start the demo. The first is the omnibase_navstack file, which is a modified version of the odometry cpp file which outputs the correct odometry data. This can be run using this command:
```
roslaunch hebiros_advanced_examples omnibase_navigation_mobile_IO.launch 
```

The second file is the Omnibase Navigation launch file. This file starts the lidar and the map server, and opens an RViz instance  to demonstrate the mapping. 

Use this command if using the HEBI-ROS repository
```
roslaunch hebiros_advanced_examples omnibase_navigation.launch 
```


Once both files are running and RViz is working, you need to set the initial position of the robot. Do this using the **2D Pose Estimate**. You can now set navigation goals using the **2D Nav Goal** tool. 

Programmatically, you can send a navigation goal by sending a message on `/move_base_simple`. The critical elements of this are setting the `frame_id` to `map`, and then filling in the position and orientation quaternion fields.  For example, this moves the robot to position (1.5, 0.25).

```
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'map'
pose:
  position:
    x: 1.5
    y: 0.25
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0" 
```

Note that the parameters for the planner, affecting speed, accuracy, etc, are given in the `mapping_navigation/param` folder.
