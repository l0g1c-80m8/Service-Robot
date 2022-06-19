#!/bin/sh

cd ../../ 

xterm -e " source devel/setup.bash; roslaunch my_robot world.launch " &

sleep 10

xterm -e " source devel/setup.bash; roslaunch my_robot amcl.launch " &

sleep 5

xterm -e " source devel/setup.bash; roslaunch my_robot navigation.launch " &

sleep 5

xterm -e " rosrun rviz rviz " & 

sleep 5

xterm -e " source devel/setup.bash; roslaunch my_robot pick_objects.launch " &