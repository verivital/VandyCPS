#bin/bash
roslaunch realsense_camera r200_nodelet_rgbd.launch & 
rosrun mavros mavros_node _fcu_url:=tcp://127.0.0.1:5760 _system_id:=2 &
rosrun flight_main flight_main_node &
rosrun cps_vision cps_vision
