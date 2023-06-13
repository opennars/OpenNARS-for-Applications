roslaunch transbot_nav laser_bringup.launch &
sleep 10
roslaunch astra_camera astrapro.launch &
roslaunch transbot_nav rrt_exploration.launch open_rviz:=true &
