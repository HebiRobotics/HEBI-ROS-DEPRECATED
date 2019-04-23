

#Rosie Navigation & Mapping Instructions


**Important: All of the mapping and navigation code on Rosie is in the lidar_workspace catkin directory; to run any of these commands, you will need to run `source devel/setup.sh` from your catkin workspace first**

# Installation & Setup

Before you can start building maps and using the navigation stack with your robot, you need to add the following ROS repositories to your /SRC folder in your Catkin workspace. These are the open source libraries used for mapping (GMapping) and navigation in ROS. They are currently compatible with ROS Kinetic and Melodic.

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

In order to use any USB-based Lidar, you need to enable access to the USB ports serial connection (/dev/ttyUSB0). The easiest way to do this is by adding your user account to the dialout group, using this command:
```
sudo gpasswd --add ${USER} dialout
```

# Rosie Mapping

In order to run the mapping demo on Rosie, there are two files that need to be run on Rosie's cpu.

The first is the omnibase_navstack file, which is a modified version of the odometry.cpp file which outputs the correct odometry data. This can be run using this command:
```
roslaunch hebiros_advanced_examples omnibase_navigation_mobile_IO.launch 
```

The second file that needs to be run is the gmapping launch file. This starts up the RPLidar module as well as RViz. Use this command:
```
roslaunch hebiros_advanced_examples rosie_gmapping.launch 
```

Using the Mobile I/O controller, drive Rosie around to build a map. When the map is finished, use the map server to save the map to the local directory with the filename FILENAME:
```
rosrun map_server map_saver -f FILENAME
```

# Rosie Navigation

In order to run the navigation demo, a couple things need to be taken care of prior.
First, you must have created a map using the gmapping demo (.pgm and .yaml file); you must set this map in the rosie_nav.launch file as the default map.
Second, the robot must have the RPLidar attached and needs to start near the origin of the map.

Two files need to be run in order to start the demo. The first is the omnibase_navstack file, which is a modified version of the odometry cpp file which outputs the correct odometry data. This can be run using this command:
```
roslaunch hebiros_advanced_examples omnibase_navigation_mobile_IO.launch 
```

The second file is the Rosie Navigation launch file. This file starts the lidar and the map server, and opens an RViz instance  to demonstrate the mapping. Use this command if using Rosie_Navigation repository:
```
roslaunch rosie_navigation rosie_nav.launch 
```
Use this command if just using HEBI-ROS repository
```
roslaunch hebiros_advanced_examples rosie_nav.launch 
```


Once both files are running and RViz is working, you need to set the initial position of the robot. Do this using the **2D Pose Estimate**. You can now set navigation goals using the **2D Nav Goal** tool. 

**Note: Although the global planner is working well, the local planner (yellow line) is still incorrect**