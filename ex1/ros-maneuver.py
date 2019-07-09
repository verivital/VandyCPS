#!/usr/bin/env python

import rospy
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import CommandTOL
import time

rospy.init_node('mavros_takeoff_python')
rate = rospy.Rate(10)

# Set Mode
print "\nSetting Mode"
rospy.wait_for_service('/mavros/set_mode')
try:
    change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    response = change_mode(custom_mode="ALT_HOLD")
    rospy.loginfo(response)
except rospy.ServiceException as e:
    print("Set mode failed: %s" %e)

# Arm
print "\nArming"
rospy.wait_for_service('/mavros/cmd/arming')
try:
    arming_cl = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    response = arming_cl(value = True)
    rospy.loginfo(response)
except rospy.ServiceException as e:
    print("Arming failed: %s" %e)

# Takeoff
print "\nTaking off"
rospy.wait_for_service('/mavros/cmd/takeoff')
try:
    takeoff_cl = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
    response = takeoff_cl(altitude=10, latitude=0, longitude=0, min_pitch=0, yaw=0)
    rospy.loginfo(response)
except rospy.ServiceException as e:
    print("Takeoff failed: %s" %e)

print "\nHovering..."
time.sleep(5)

# Land
print "\nLanding"
rospy.wait_for_service('/mavros/cmd/land')
try:
    takeoff_cl = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
    response = takeoff_cl(altitude=10, latitude=0, longitude=0, min_pitch=0, yaw=0)
    rospy.loginfo(response)
except rospy.ServiceException as e:
    print("Landing failed: %s" %e)

# Disarm
print "\nDisarming"
rospy.wait_for_service('/mavros/cmd/arming')
try:
    arming_cl = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    response = arming_cl(value = False)
    rospy.loginfo(response)
except rospy.ServiceException as e:
    print("Disarming failed: %s" %e)
