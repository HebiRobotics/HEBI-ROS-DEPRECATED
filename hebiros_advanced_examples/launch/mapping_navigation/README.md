# Omnibase Navigation & Mapping Instructions


**Important: In order to run the Navigation & Mapping code, you must already have a catkin workspace setup with omnibase code running using the HEBI-ROS and the hebi-cpp-api-ros repositories. Please visit this link for setup instruction: https://github.com/HebiRobotics/HEBI-ROS/tree/develop/hebiros_advanced_examples/src/omnibase**


# Installation & Setup

Before you can start building maps and using the navigation stack with your robot, you need to add the following ROS repositories to your `src` folder in your catkin workspace. These are the open source libraries used for mapping (GMapping) and navigation in ROS. They are currently compatible with ROS Kinetic and Melodic.

```
git clone https://github.com/ros-perception/openslam_gmapping

git clone https://github.com/ros-perception/slam_gmapping.git

git clone https://github.com/ros-planning/navigation.git

git clone https://github.com/ros/geometry2.git

git clone https://github.com/ros-planning/navigation_msgs.git
```

In addition to the source code, there are a couple of libraries that need to be installed on your machine. Please make sure the following are installed by running these commands:
```
sudo apt-get install libsdl1.2-dev
sudo apt-get install libsdl-image1.2-dev
```

### FOR KINETIC ONLY -- FIX BUILD OF NAVIGATION PACKAGE!
```
cd navigation
git checkout kinetic-devel
rm -r fake_localization
cd ..
```
## RPLidar Setup

### Installation

To use the RP Lidar system with the omnibase, you need to clone the source code into your catkin workspace. From your workspace directory, run these commands:
```
cd src
git clone https://github.com/robopeak/rplidar_ros.git
cd ..
catkin_make
```

### USB Configuration

In order to use any USB-based Lidar, you need to enable access to the USB ports serial connection (/dev/ttyUSB0). The easiest way to do this is by adding your user account to the dialout group, using this command:
```
sudo gpasswd --add ${USER} dialout
```

### Filtering

The laser filtering is implemented with the `laser_filters` packages (wiki.ros.org/laser_filters), which reads from topic `scan` and publishes filtered data to `scan_filtered`.
This package is used to modify incoming scan data from the laser sensor used on the robot. **his package is not necessary if you have no reason to modify the laser data (i.e. no obstructions).**

To install the laser filters, its recommended to install from APT, for your specific linux distribution:
```
sudo apt install ros-<distro>-laser-filters
```
You can also download the laser filters source code from the Github repository, and place it in your catkin workspace:
```
https://github.com/ros-perception/laser_filters.git
```
The two useful filters we have for the omnibase are the "Angular Bounds" and "Angular Bounds In Place" filters. The Angular Bounds Filter has been created in order to ignore laser scans outside of a certain field of view. The bounds of the obstruction can be set with `lower_angle` and `upper_angle`. The Angular Bounds In Place Filter has been created in order to ignore laser scans within a certain subset of the field of view. The bounds of the obstruction can be set with `lower_angle` and `upper_angle`.

The parameters for a default setup (ignoring data in a 60 degree arc behind the robot) is found in `mapping_navigation/params/laser_filters.yaml`. 
In order to configure the out-coming scan topic for the laser filter package, you can change the name of the publish topic directly in the .launch and .yaml files for the specific filter.

To test and tune, you can run the "mapping" node below, and watch the laser returns; this filter is automatically applied (via the launch files) when running the mapping and navigation nodes below.
Use of the laser filter package is **not required** for mapping to work; it is just an additional tool made available with this software.

### Mounting Position / Configuration

The relative position of the LIDAR is stored in the `hebiros_description/urdf/omnibase.xacro` file; find the `scan_joint` joint near the bottom of the file, and set the x/y/z and r/p/y to match the mounting position (relative to the center of the base frame)


### Launch Configurations for Laser Filter

If your omnibase node will be using the filtered laser data, you will need to change the scan topic inputs for the gmapping and amcl files. 

To use the filtered laser data for mapping, the default scan topic needs to be changed on line 3 to the new scan topic, such as `scan_filtered`.

To use the filtered laser data for navigation (localization), the default scan topic needs to be changed on line 3 of the amcl.launch file, from `scan` to `scan_filtered` (or your custom filter name).


In order to view the modified scan data in RViz, you will need to either manually change the LaserScanner topic in the left panel to the new `scan_filtered` topic, or you will need to modify line *87* in the omnibase_simple_gazebo_model.rviz file and line *125* in the omnibase_navigation.rviz file.

## Rosie Mapping

In order to run the mapping demo on the omnibase, there are two files that need to be run on omnibase's CPU.

The first is the omnibase_teleop_mobile_IO file, initializes the omnibase node and starts publishing live odometry to the `/odom` topic; this also enables the Mobile I/O controller. This can be run using this command:
```
roslaunch hebiros_advanced_examples omnibase_teleop_mobile_IO.launch 
```

The second file that needs to be run is the mapping launch file. This starts up the RPLidar module as well as RViz. Use this command:
```
roslaunch hebiros_advanced_examples omnibase_gmapping.launch 
```

If you plan on using a laser filter to modify the scan data, you also need to launch the filter (with name <filtername>). This can be done using this command:
```
roslaunch hebiros_advanced_examples <filtername>.launch
```

Using the Mobile I/O controller, drive the omnibase around to build a map. When the map is finished, use the map server to save the map to the local directory with the filename FILENAME:
```
rosrun map_server map_saver -f FILENAME
```

(Note: running the map_saver command should be done in the same directory which you are saving the map to; if you try to specify a directory or path in the "FILENAME", you will need to manually edit the yaml file after saving to fix the relative path reference between yaml and pgm files)

### Tuning Mapping

To tune the speed and accuracy of the mapping demo using the GMapping library, you can change some of the key parameters found in the omnibase_gmapping launch file. Here are the recommended parameters to start with:

`minimumScore`: This sets the minimum score for considering the outcome of the scan matching good (default: 50)

`linearUpdate`: Increase this value to allow more time between updates from the laser (default: 0.1)

`particles`: Change this value to set the number of particles used in the filter (default: 100)

For a full list of parameters and their relevance to mapping, please refer to the GMapping wiki: http://wiki.ros.org/gmapping


## Rosie Navigation

In order to run the navigation demo, a couple things need to be taken care of prior.
First, you must have created a map using the gmapping demo (.pgm and .yaml files); you must set this map in the omnibase_navigation.launch file as the default map. On line 3 of the omnibase_navigation.launch file, make sure you replace "default_map.yaml" with you preferred input map.
Second, the robot must have the RPLidar attached and needs to start near the origin of the map. If the start position is a considerable distance from the map's origin, the robot will require an `initial_pose` message to be provided (can be done using RViz), explained below.

Two files need to be run in order to start the demo. The first is the omnibase_teleop_mobile_IO file, initializes the omnibase node and starts publishing live odometry to the `/odom` topic; this also enables the Mobile I/O controller. This can be run using this command:
```
roslaunch hebiros_advanced_examples omnibase_teleop_mobile_IO.launch 
```

The second file is the Omnibase Navigation launch file. This file starts the RP Lidar and the map server, and opens an RViz instance to demonstrate the mapping. 
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
### Tuning Navigation

In order to customize and tune the navigation stack, you can modify the parameter files found in the `mapping_navigation/param` folder. These parameters affect speed, accuracy, localization, and population of the costmaps.

For a full list of parameters and their relevance to the navigation, please refer to the following wiki pages:

http://wiki.ros.org/dwa_local_planner#Parameters                                                  
http://wiki.ros.org/base_local_planner#Parameters                                                          
http://wiki.ros.org/amcl#Parameters                                                                       
http://wiki.ros.org/move_base#Parameters



