#!/bin/bash
sudo add-apt-repository http://packages.ros.org/ros/ubuntu
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 0xB01FA116
sudo apt-get update
sudo apt -y install ros-kinetic-desktop-full ros-kinetic-rqt python-rosinstall ros-kinetic-realsense-camera ros-kinetic-mavros ros-kinetic-web-video-server ros-kinetic-visp-tracker ros-kinetic-visp-camera-calibration ros-kinetic-vision-visp ros-kinetic-vision-opencv ros-kinetic-video-stream-opencv ros-kinetic-uvc-camera ros-kinetic-usb-cam ros-kinetic-test-mavros ros-kinetic-rviz-visual-tools ros-kinetic-rostopic ros-kinetic-roslaunch python-rosinstall python-rosinstall-generator python-wstool build-essential ros-kinetic-pyros python-rosdep
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo geographiclib-get-geoids egm96-5
cd 
if [!-d catkin_ws]
then
	source /opt/ros/kinetic/setup.bash
	mkdir ~/catkin_ws/src
	cd ~/catkin_ws/
	catkin_make
fi
cp -a ~/competition_packages/. ~/catkin_ws/src/
cd ~/catkin_ws/
catkin_make
sudo reboot