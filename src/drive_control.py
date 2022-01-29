#!/usr/bin/env python
# author Adon Sharp
# version 0.2
# date 1-15-2022
# warning this file contains temporary implementations for cameras, tcu board control, and copilot interface
# mainpage the drive_control node
# section intro_sec Introduction
# This code contains implementations for bilinear control, sensitivity, and 4-way inversion. The node subscribes to a joy topic and publishes rov/cmd_vel to PID algorithms and vector drive.
# section compile_sec Compilation
# Compile using catkin_make in the ros_workspace directory.

import rospy
from geometry_msgs import  Twist
from sensor_msgs import Joy

from std_msgs import UInt8  # For camera  pub
from std_msgs import Bool  # For TCU relay and solenoid controller pub and for pids
from std_msgs import Float64  # For pids
from nav_msgs import Odometry

from math import copysign
# need to import dynamic reconfigure
# need to import copilot interface
# need to import rov sensitivity


linearJoyAxisFBIndex = 1  # forward-backward axis index in the joy topic array from the logitech Extreme 3D Pro
linearJoyAxisLRIndex = 0  # left-right axis index in the joy topic array from the logitech Extreme 3D Pro
angularJoyAxisIndex = 2  # rotational axis index in the joy topic array from the logitech Extreme 3D Pro
verticalJoyAxisIndex = 3  # vertical axis index in the joy topic array from the logitech Extreme 3D Pro

verticalThrottleAxis = 2  # vertical axis index in the joy topic array from the Thrustmaster TWCS Throttle

l_scale = 0.5  # Holds a percent multiplier for sensitivity control. Default = 50%
a_scale = 0.5  # Holds a percent multiplier for sensitivity control. Default = 50%
v_scale = 0.5  # Holds a percent multiplier for sensitivity control. Default = 50%

a_axis = 0  # Holds the value of the rotational/angular control axis
l_axisLR = 0  # Holds the value of the right-left linear control axis
l_axisFB = 0  # Holds the value of the front-back linear control axis
v_axis = 0  # Holds the value of the vertical control axis
dh_eff = 0  # Holds depth hold PID control effort


thrustEN = False  # thrusters enabled (True = yes, False = default = no

microEN = False


useJoyVerticalAxis = True  # Holds the state that determines whether the joysticks vertical input of the throttles vertical input gets used

# inversion -> 1 Front, 2 Left, 3 Back, 4 Right, 5 Flipped; used when switching perspectives on ROV
inversion = 0

# Variable for determining the bilinear threshold
bilinearRatio = 1.5

# At what percent of the joysticks axis magnitude (-1 to 1) to apply the additional thrust
bilinearThreshold = 1 / bilinearRatio

# Exponent for Drive Power Calculations
driveExp = 1.4

dhMostRecentDepth = 0  # holds depth for when depth hold needs to be enabled
dhEnable = False

roll_cmd_vel = 0  # global to store roll_stab control effort for cmd_vel integration (there has to be a better way)




def expDrive(axis):
  axis = copysign(abs(axis) ** driveExp)
  return axis

# variable for monitoring the topic frequency so that a disconnect can be declared if the frequency drops below 1Hz
joyHorizontalLastInput = 0.0

def joyHorizontalCallback(joy):
    global joyHorizontalLastInput, a_axis, l_axisLR, l_axisFB, v_axis, dhEnable
    joyHorizontalLastInput = rospy.get_time()
    # check if thrusters disabled
    if thrustEN:
        # joystick message
        if inversion == 5:
            a_axis = joy.axes[angularJoyAxisIndex] * a_scale
        else:
            a_axis = joy.axes[angularJoyAxisIndex] * a_scale * -1

        # NOTE: right and rotate right are negative on the joystick's LR axis
        # multiple LR axis by -1 in base position (front-front, etc.)to make right positive

        if inversion == 1:  # right side is front
            l_axisFB = joy.axes[linearJoyAxisLRIndex] * l_scale * -1
            l_axisLR = joy.axes[linearJoyAxisFBIndex] * l_scale
        elif inversion == 2:  # back side is front
            l_axisLR = joy.axes[linearJoyAxisLRIndex] * l_scale
            l_axisFB = joy.axes[linearJoyAxisFBIndex] * l_scale * -1
        elif inversion == 3:  # left side is front
            l_axisFB = joy.axes[linearJoyAxisLRIndex] * l_scale
            l_axisLR = joy.axes[linearJoyAxisFBIndex] * l_scale * -1
        else:  # front side is front
            l_axisLR = joy.axes[linearJoyAxisLRIndex] * l_scale * -1
            l_axisFB = joy.axes[linearJoyAxisFBIndex] * l_scale

        # apply the exponential ratio on all axis
        a_axis = expDrive(a_axis)
        l_axisLR = expDrive(l_axisLR)
        l_axisFB = expDrive(l_axisFB)
        if useJoyVerticalAxis:
            v_axis = joy.axes[verticalJoyAxisIndex] * v_scale * -1
            v_axis = expDrive(v_axis)

            # turn position-based depth hold on/off
            dhEnableMsg = Bool()
            if v_axis == 0:
                dhEnable = True
                dhEnableMsg.data = dhEnable
            else:
                dhEnable = False
                dhEnableMsg.data = dhEnable

            dh_enable_pub.publish(dhEnableMsg)
    else:
        a_axis = 0
        l_axisLR = 0
        l_axisFB = 0
        v_axis = 0
        dhEnable = False

def joyVertivalCallback(joy):
  # once copilot interface is created the params will be replaced with topics (inversion + sensitivity)
  joyVerticalLastInput = rospy.get_time()
  # check if thrusters disabled
  useJoyVerticalAxis = False
  if thrustEN:
    v_axis = expDrive()
    v_axis = expDrive()


rospy.init_node("drive_control")
joy_sub1 = rospy.Subscriber('joy/joy1', Joy, 2)
joy_sub2 = rospy.Subscriber('joy/joy2', Joy, 2)
thruster_status_sub = rospy.Subscriber('rov/thruster_status', Bool, 1)
sensitivity_sub = rospy.Subscriber('rov/sensitivity', rov_sensitivity, 3)
dh_state_sub = rospy.Subscriber('odometry/filtered', Odometry, 1)
dh_ctrl_eff_sub = rospy.Subscriber('depth_hold/control_effort', Float64, 1)
dh_toggle_sub = rospy.Subscriber('depth_hold/pid_enable', Bool, 1)
rs_ctrl_eff_sub = rospy.Subscriber('roll_stabilization/control_effort', Float64, 1)
inversion_sub = rospy.Subscriber('rov/inversion', UInt8, 1)

vel_pub = rospy.Publisher('rov/cmd_vel', Twist, 1)
camera_select = rospy.Publisher('rov/camera_select', UInt8, 3)
power_control = rospy.Publisher('tcu/main_relay', Bool, 3)
solenoid_control = rospy.Publisher('tcu/main_solenoid', Bool, 3)
sensitivity_pub = rospy.Publisher('rov/sensitivity', rov_sensitivity, 3)
thruster_status_pub = rospy.Publisher('rov/thruster_status', Bool, 3)
micro_status_pub = rospy.Publisher('micro/enable', Bool, 3)
dh_cmd_vel_pub = rospy.Publisher('rov/cmd_vel', Twist, 1)
dh_setpoint_pub = rospy.Publisher('depth_hold/setpoint', Float64, 1)
dh_enable_pub = rospy.Publisher('depth_hold/pid_enable', Bool, 1)

# topics for PIDs
lat_pid_pub = rospy.Publisher('/lat_motion/pid_enable', Bool, 1)
long_pid_pub = rospy.Publisher('/long_motion/pid_enable', Bool, 1)
vert_pid_pub = rospy.Publisher('/vert_motion/pid_enable', Bool, 1)
roll_pid_pub = rospy.Publisher('/roll_motion/pid_enable', Bool, 1)
yaw_pid_pub = rospy.Publisher('/yaw_motion/pid_enable', Bool, 1)