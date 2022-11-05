#!/usr/bin/env python
# author Adon Sharp | v0.2 | 1-15-2022
# author Alex Bertran | v1.0 | 10-29-2022
# warning this file contains temporary implementations for cameras, tcu board control, and copilot interface
# mainpage the drive_control node
# section intro_sec Introduction
# This code contains implementations for bilinear control, sensitivity, and 4-way inversion. The node subscribes to a joy topic and publishes rov/cmd_vel to PID algorithms and vector drive.
# section compile_sec Compilation
# Compile using catkin_make in the ros_workspace directory.

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

from std_msgs.msg import UInt8  # For camera  pub
from std_msgs.msg import Bool  # For TCU relay and solenoid controller pub and for pids
from std_msgs.msg import Float64  # For pids
from nav_msgs.msg import Odometry

from math import copysign
from dynamic_reconfigure.server import Server
from copilot_interface.cfg import copilotControlParamsConfig
from rov_control_interface.msg import rov_sensitivity

rospy.init_node("drive_control")
linearJoyAxisFBIndex = 1  # forward-backward axis index in the joy topic array from the logitech Extreme 3D Pro
linearJoyAxisLRIndex = 0  # left-right axis index in the joy topic array from the logitech Extreme 3D Pro
angularJoyAxisIndex = 2  # rotational axis index in the joy topic array from the logitech Extreme 3D Pro
verticalJoyAxisIndex = 3  # vertical axis index in the joy topic array from the logitech Extreme 3D Pro
verticalThrottleAxis = 2  # vertical axis index in the joy topic array from the Thrustmaster TWCS Throttle

sensitivity = {"linear": 0.5, "angular": 0.25, "vertical": 0.5} #Holds a percent multiplier for ROV sensitivity

#COME BACK AND REMOVE THESE
l_scale = 0.5  # Holds a percent multiplier for sensitivity control. Default = 50%
a_scale = 0.25  # Holds a percent multiplier for sensitivity control. Default = 50%
v_scale = 0.5  # Holds a percent multiplier for sensitivity control. Default = 50%

thrustEN = False  # thrusters enabled (True = yes, False = default = no
dhEnable = False  # for depth hold, TEMPORARY PLACEHOLDER TO PREVENT ERRORS

# inversion -> 1 Front, 2 Left, 3 Back, 4 Right, 5 Flipped; used when switching perspectives on ROV
inversion = 0 # DELETE LATER

# Exponent for Drive Power Calculations
driveExp = 1.4

#The vector that gets edited by the callbacks and then published
joyVector = Twist()

# Multiplies the value of the axis by an exponent
# Uses copysign to make sure that raising axis to an even power can still return a negative number
def expDrive(axis):
  axis = copysign(abs(axis) ** driveExp, axis)
  return axis

def joyHorizontalCallback(joy):
    global joyVector, sensitivity

    # If thrusters enabled, map the joystick inputs to the joyVector
    if thrustEN:
        # multiply LR axis by -1 in base position (front-front, etc.)to make right positive
        # NOTE: right and rotate right are negative on the joystick's LR axis
        a_axis = joy.axes[angularJoyAxisIndex] * sensitivity['angular'] * -1
        l_axisLR = joy.axes[linearJoyAxisLRIndex] * sensitivity['linear'] * -1 
        l_axisFB = joy.axes[linearJoyAxisFBIndex] * sensitivity['linear']

        # apply the exponential ratio on all axis
        a_axis = expDrive(a_axis)
        l_axisLR = expDrive(l_axisLR)
        l_axisFB = expDrive(l_axisFB)

    else:
        a_axis = 0
        l_axisLR = 0
        l_axisFB = 0

    joyVector.linear.x = l_axisLR
    joyVector.linear.y = l_axisFB
    joyVector.angular.x = a_axis

    vel_pub.publish(joyVector)

# Callback that runs whenever the throttle sends an update
def joyVerticalCallback(joy):
  global joyVector
  
  # check if thrusters disabled
  if thrustEN:
    v_axis = joy.axes[verticalThrottleAxis] * sensitivity["vertical"] * -1
    v_axis = expDrive(v_axis)
  else:
    v_axis = 0

  joyVector.linear.z = v_axis
  vel_pub.publish(joyVector)

# Handles copilot input: updates thrusters, enables sensitivity, and enables inversion.
# Callback to anything published by the dynamic reconfigure copilot page

def controlCallback(config, level):
    global thrustEN, sensitivity, l_scale, a_scale, v_scale, dhEnable, p_scalar, i_scalar, d_scalar
    thrustEN = config.thrusters

    sensitivity['linear'] = config.l_scale
    sensitivity['angular'] = config.a_scale
    sensitivity['vertical'] = config.v_scale
    
    p_scalar = config.p_scalar
    i_scalar = config.i_scalar
    d_scalar = config.d_scalar
    dhEnable = config.dh_enable

    # Sensitivity publisher
    sensitivityMsg = rov_sensitivity()
    sensitivityMsg.l_scale = l_scale
    sensitivityMsg.a_scale = a_scale
    sensitivityMsg.v_scale = v_scale
    sensitivity_pub.publish(sensitivityMsg)    
    
    # Thrusters enabled Publisher
    thrusterStatusMsg = Bool()
    thrusterStatusMsg.data = thrustEN
    thruster_status_pub.publish(thrusterStatusMsg)

    # Inversion Publisher
    inversionMsg = UInt8()
    inversionMsg.data = config.inversion
    inversion_pub.publish(inversionMsg)
    return config

# What the node does when copilot inversion setting publishes a new message
# joy "sensor_msgs/Joy" message that is received when the joystick publishes a new message

def inversionCallback(data):
  global inversion
  inversion = data.data
  
# What the node does when copilot sensitivity setting publishes a new message
# "rov_control_interface/rov_sensitivity." message that is received when the sensitivity setting is changed

def sensitivityCallback(data):
    global l_scale, a_scale, v_scale
    l_scale = data.l_scale
    a_scale = data.a_scale
    v_scale = data.v_scale

def depthHoldCallback(data):
    global p_scalar, i_scalar, d_scalar
    p_scalar = data.p_scalar
    i_scalar = data.i_scalar
    d_scalar = data.d_scalar

# What the node does when thruster status topic publishes a new message
# "std_msgs/Bool" message that is received when the topic publishes a new message

def ROS_INFO_STREAM(thrustEN):
    pass

def thrusterStatusCallback(data):
  global thrustEN
  thrustEN = data.data
  ROS_INFO_STREAM(thrustEN)

def dhToggleCallback(data):
  global dhEnable
  dhEnable = data.data

def dhStateCallback(data):
  global dhMostRecentDepth
  if dhEnable: # only update depth if depth hold is disabled {dhEnable == False}
      dhMostRecentDepth = data.pose.pose.position.z * -1
      depth = Float64()
      depth.data = dhMostRecentDepth
      dh_setpoint_pub.publish(depth)

def dhControlEffortCallback(data): # no need for dhEnable check since PIDs won't publish control effort when disabled
  global dh_eff
  dh_eff = data.data

def main():
    global joy_sub1, joy_sub2, thruster_status_sub, sensitivity_sub, depth_hold_sub, dh_state_sub, dh_ctrl_eff_sub, dh_toggle_sub, rs_ctrl_eff_sub, inversion_pub, inversion_sub, vel_pub, camera_select, power_control, solenoid_control, sensitivity_pub, depth_hold_pub, thruster_status_pub, micro_status_pub, dh_cmd_vel_pub, dh_setpoint_pub, dh_enable_pub
    joy_sub1 = rospy.Subscriber('joy/joy1', Joy, joyHorizontalCallback)
    joy_sub2 = rospy.Subscriber('joy/joy2', Joy, joyVerticalCallback)
    thruster_status_sub = rospy.Subscriber('rov/thruster_status', Bool,thrusterStatusCallback)
    sensitivity_sub = rospy.Subscriber('rov/sensitivity', rov_sensitivity, sensitivityCallback)
    #depth_hold_sub = rospy.Subscriber('depth_hold/pid_enable', PID, depthHoldCallback)
    dh_state_sub = rospy.Subscriber('odometry/filtered', Odometry, dhStateCallback)
    dh_ctrl_eff_sub = rospy.Subscriber('depth_hold/control_effort', Float64, dhControlEffortCallback)
    dh_toggle_sub = rospy.Subscriber('depth_hold/pid_enable', Bool, dhToggleCallback)
    inversion_sub = rospy.Subscriber('rov/inversion', UInt8, inversionCallback)

    vel_pub = rospy.Publisher('rov/cmd_vel', Twist, queue_size=1)
    inversion_pub = rospy.Publisher('rov/inversion', UInt8, queue_size=1)
    camera_select = rospy.Publisher('rov/camera_select', UInt8, queue_size=3)
    power_control = rospy.Publisher('tcu/main_relay', Bool, queue_size=3)
    solenoid_control = rospy.Publisher('tcu/main_solenoid', Bool, queue_size=3)
    sensitivity_pub = rospy.Publisher('rov/sensitivity', rov_sensitivity, queue_size=3)
    #depth_hold_pub = rospy.Publisher('depth_hold/pid_vals', PID, queue_size=3)
    thruster_status_pub = rospy.Publisher('rov/thruster_status', Bool, queue_size=3)
    dh_setpoint_pub = rospy.Publisher('depth_hold/setpoint', Float64, queue_size=1)
    dh_enable_pub = rospy.Publisher('depth_hold/pid_enable', Bool, queue_size=1)

    # setup dynamic reconfigure
    server = Server(copilotControlParamsConfig, controlCallback)

    # Enter the event loop
    rospy.spin()

if __name__  == "__main__":
    main()
