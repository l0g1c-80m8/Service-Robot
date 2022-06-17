#!/bin/sh

xterm -e " roslaunch my_robot world.launch " &

sleep 10

xterm -e " roslaunch my_robot slam_gmapping.launch " &

sleep 5

xterm -e " roslaunch my_robot view_navigation.launch " &

sleep 5

xterm -e " roslaunch my_robot keyboard_teleop.launch " &