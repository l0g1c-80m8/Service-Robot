#!/bin/sh

# Move to catkin_ws root
cd ../../

# Launch the world containing the robot
xterm -e " source devel/setup.bash; roslaunch my_robot world.launch " &

sleep 10

# Launch gmapping package for SLAM
xterm -e " source devel/setup.bash; roslaunch my_robot slam_gmapping.launch " &

sleep 5

# Launch rviz for navigation
xterm -e " source devel/setup.bash; roslaunch my_robot view_navigation.launch rviz_config:=\"./rvizConfig/viz-test-slam.rviz\"" &

sleep 5

# Launch teleop node to manually control the robot
xterm -e " source devel/setup.bash; roslaunch my_robot keyboard_teleop.launch " &