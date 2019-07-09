#pull from the osrf full kinetic build
FROM nvidia/cudagl:9.0-base-ubuntu16.04
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES},display

#install ros
RUN apt-get update && apt-get install -q -y \
    dirmngr \
    gnupg2 \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    && rm -rf /var/lib/apt/lists/*

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# bootstrap rosdep
RUN rosdep init \
    && rosdep update

# install ros packages
ENV ROS_DISTRO kinetic
RUN apt-get update && apt-get install -y ros-kinetic-desktop-full && rosdep update && apt install -y python-rosinstall python-rosinstall-generator python-wstool build-essential


#install some ros packages we are gonna need
RUN apt-get update && apt-get install -y ros-kinetic-driver-base  ros-kinetic-navigation ros-kinetic-ros-control ros-kinetic-teb-local-planner ros-kinetic-move-base ros-kinetic-ros-controllers ros-kinetic-gazebo-ros-control ros-kinetic-ackermann-msgs ros-kinetic-joy \
    ros-kinetic-mavros ros-kinetic-mavros-extras ros-kinetic-mavros-msgs 
#navigate to home directory

RUN apt-get remove modemmanager -y && apt-get update && apt-get install -y git zip qtcreator cmake build-essential genromfs ninja-build exiftool astyle &&  apt-get install -y protobuf-compiler libeigen3-dev libopencv-dev && \ 
    apt-get -y install python-argparse python-empy python-toml python-numpy python-dev python-pip && \ 
    pip install pandas jinja2 pyserial pyyaml && pip install pyulog 
WORKDIR home 

COPY install_geographiclib_datasets.sh install_geographiclib_datasets.sh
RUN /bin/bash -c "chmod +x install_geographiclib_datasets.sh && ./install_geographiclib_datasets.sh" 

RUN git clone https://github.com/PX4/Firmware.git

RUN mkdir -p catkin_ws/src 
WORKDIR catkin_ws
#intialize the workspace
RUN /bin/bash -c 'source /opt/ros/kinetic/setup.bash && catkin_make'
#navigate into the src directory
WORKDIR src
COPY ex1 ex1 
WORKDIR ..
RUN /bin/bash -c 'source /opt/ros/kinetic/setup.bash && catkin_make && source devel/setup.bash'
WORKDIR ..


CMD /bin/bash -c "cd Firmware && make px4_sitl gazebo"
# RUN mkdir -p catkin_ws/src 
# WORKDIR catkin_ws
# #intialize the workspace
# RUN /bin/bash -c 'source /opt/ros/kinetic/setup.bash && catkin_make'