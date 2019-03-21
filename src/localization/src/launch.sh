#!/usr/bin/env bash
source ~/contrib/Robotcup2018/2018-5-7/devel/setup.bash
rosrun image_view image_view image:=zed/left/image_rect_color &
roslaunch localization localization_testing.launch &
rosbag play ../../../2018-05-05-15-21-15.bag --clock -r 0.5 &

