# VandyCPS
Hello and welcome to our repository! This repo was built for the Vanderbilt team competitng in the 2018 NSF Student CPS Challenge
"No robot left behind!" (more information about the competition can be found at this site: [https://cps-vo.org/group/CPSchallenge.](https://cps-vo.org/group/CPSchallenge "2018 NSF Student CPS Challenge")

The team consists of the following members and was led by Nate Hamilton
Graduate Students: <br />
Ran Hao <br />
Patrick Musau
**PLEASE ADD YOUR NAMES**
Undergraduate Students: **PLEASE ADD YOUR NAMES**

Hopefully you won't get too lost as I try to explain how this is set up.

## Organization
The repo is split up into 2 different folders. 
1. The folder _setup_ contains bash scripts and a make file for setting up the Intel Aero RTF drone from a fresh Ubuntu distro. For more information about that, scroll down the the __Setup__ section.
2. The folder _competition_packages_ contains the ros node code used in the competition as rospackages. If you already have a catkin workspace, moving one of the folders within this section to the _src_ folder of your workspace and running `catkin_make` will allow you to use the package. For more information about that and the contents of each package, scroll down the the __Package Information__ section.

## Important Information for Use
**should list all of the special commands and settings necessary for use**

## Setup
In order to get the drone to this point, follow the directions from [02 Initial Setup](https://github.com/intel-aero/meta-intel-aero/wiki/02-Initial-Setup "Initial Setup") and, subsequently, [Instaling Ubuntu on Aero.](https://github.com/intel-aero/meta-intel-aero/wiki/90-(References)-OS-user-Installation "Instaling Ubuntu on Aero")
After you have done that, clone this repository into the home directory, and run the following lines of code:

```
cd ~/setup
sudo make all
```

Since there is a reboot command after each step in the installation, you will have to run the following commands once the system reboots.
`cd ~/setup && sudo make 2` and `cd ~/setup && sudo make 3` after the second reboot.

## Package Information
Each folder in this section is a self-contained rospackage. Copying one of the folders into the _src_ folder of your workspace and running `catkin_make` will allow you to use the package. For your convenience, running the setup code will copy all of the packages into a catkin workspace labeld _catkin_ws_ in your home directory. For information about each package, read below.

### flight_test
This is a simple test we used for making sure the navigation controller worked. The drone should fly up 1.5m into the air, do a tight square with 0.1m side lengths, and then land.
