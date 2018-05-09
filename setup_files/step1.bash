#!/bin/bash
echo 'deb https://download.01.org/aero/deb xenial main' | sudo tee /etc/apt/sources.list.d/intel-aero.list
wget -qO - https://download.01.org/aero/deb/intel-aero-deb.key | sudo apt-key add -
sudo apt-get update
sudo apt-get upgrade
sudo apt-get -y install gstreamer-1.0 libgstreamer-plugins-base1.0-dev libgstrtspserver-1.0-dev gstreamer1.0-vaapi gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-libav ffmpeg v4l-utils python-pip
sudo pip install pymavlink
sudo apt-get -y install aero-system
sudo reboot