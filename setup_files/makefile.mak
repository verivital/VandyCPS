all install:
	chmod +x step1.bash step2.bash
	sudo ./step1.bash
	sudo ./step2.bash
	
1:
	chmod +x step1.bash step2.bash
	sudo ./step1.bash

2:
	chmod +x step1.bash step2.bash
	sudo ./step2.bash

clean:
	chmod -x step1.bash step2.bash

play:	
	roscore & roscd realsense_camera & roslaunch realsense_camera r200_nodelet_rgbd.launch & rosrun mavros mavros_node _fcu_url:=tcp://127.0.0.1:5760 _system_id:=2
