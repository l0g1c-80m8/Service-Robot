#!/bin/sh

# Move to catkin_ws root
cd ../../

# Launch the world containing the robot
xterm -e " source devel/setup.bash; roslaunch my_robot world.launch " &

sleep 10

# Launch amcl package for localization
xterm -e " source devel/setup.bash; roslaunch my_robot amcl.launch " &

sleep 5

# Launch rviz to localize the results
xterm -e " source devel/setup.bash; roslaunch my_robot view_navigation.launch " &

sleep 5

# Launch add_markers to vizualize objects in rviz
xterm -e " source devel/setup.bash; roslaunch my_robot add_markers.launch " &