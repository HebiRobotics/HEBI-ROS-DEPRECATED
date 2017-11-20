Running examples using MoveIt!:


Setup:

- Start by installing hebiros
    http://wiki.ros.org/hebiros

- Either follow this tutorial to create a MoveIt! package with SRDF or
    http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/setup_assistant/setup_assistant_tutorial.html

- Use the provided folder "x_demo" which is an example of a 3-DOF-arm

- The package should be placed adjacent to hebiros and built using catkin_make


Visualtization:

- Run either of the following to start the MoveIt! GUI
    roslaunch <my_package> demo.launch
    roslaunch x_demo demo.launch

- You can try planning and executing paths to different positions
    http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/ros_visualization/visualization_tutorial.html


Robot control:

- If you have a physical robot set up, you can use MoveIt! to control it

- Leave the visualization running

- Start the hebiros node
    rosrun hebiros hebiros_node

- Run the example which reads feedback from MoveIt! and sends it as a command to the robot
    roslaunch hebiros example_moveit_node

- The robot will now follow paths when executed in MoveIt!
