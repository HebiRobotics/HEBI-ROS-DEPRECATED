# System setup and Installation

To install the dependencies, build, and run the IROS 2018 ROSie demo on a new system, follow the following instructions.

This process can be used if running ROSie code on a HEBI Rosie kit.

## Dependency Installation:

# (1) Install Ubuntu 18.04

# (2) Install librealsense drivers (v2.17)

To do this, follow Step 1 at https://github.com/intel-ros/realsense/#installation-instructions

We used v2.17, although see notes on ROS realsense wrapper installation for necessary workarounds.  Other versions may work as well.

# (3) Install ROS (Melodic)

(Below instructions taken from http://wiki.ros.org/melodic/Installation/Ubuntu)

```sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt update

sudo apt install ros-melodic-desktop-full

sudo rosdep init
rosdep update

echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential```

# (4) Create ros workspace + install ros realsense package

(Below instructions adapted from https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md)

```cd ~
mkdir rosie_workspace
cd rosie_workspace
mkdir src
cd src```

```wget https://github.com/intel-ros/realsense/archive/2.1.1.tar.gz
tar -xzvf 2.1.1.tar.gz
rm 2.1.1.tar.gz```

Patches needed for V2.1.1 of the ROS wrapper in combination with v2.17 of the librealsense2 drivers:
* https://github.com/intel-ros/realsense/pull/523/files
* white balance patch found at https://github.com/intel-ros/realsense/issues/496 (comment out case statement in noted file)

```catkin_init_workspace 
cd ..
catkin_make clean
catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
catkin_make install
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc```

# (5) Download code

```cd src
git clone https://github.com/HebiRobotics/HEBI-ROS.git
cd HEBI-ROS
git checkout iamtesch/ROSie```


# (6) Build

```cd ../..
catkin_make example_nodes_generate_messages
catkin_make```
