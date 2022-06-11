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

l_scale = 0.5  # Holds a percent multiplier for sensitivity control. Default = 50%
a_scale = 0.25  # Holds a percent multiplier for sensitivity control. Default = 50%
v_scale = 0.5  # Holds a percent multiplier for sensitivity control. Default = 50%

p_scalar = 0
i_scalar = 0
d_scalar = 0

a_axis = 0  # Holds the value of the rotational/angular control axis
l_axisLR = 0  # Holds the value of the right-left linear control axis
l_axisFB = 0  # Holds the value of the front-back linear control axis
v_axis = 0  # Holds the value of the vertical control axis
dh_eff = 0  # Holds depth hold PID control effort


thrustEN = False  # thrusters enabled (True = yes, False = default = no

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
  axis = copysign(abs(axis) ** driveExp, axis)
  return axis

# variable for monitoring the topic frequency so that a disconnect can be declared if the frequency drops below 1Hz
joyHorizontalLastInput = 0.0

def joyHorizontalCallback(joy):
    global joyHorizontalLastInput, a_axis, l_axisLR, l_axisFB, v_axis, dhEnable, commandVectors, camera_select
    if joy.buttons[2]:
        camera_select.publish(1)
    elif joy.buttons[3]:
        camera_select.publish(2)
    elif joy.buttons[4]:
        camera_select.publish(3)
    elif joy.buttons[5]:
        camera_select.publish(4)
    
    joyHorizontalLastInput = rospy.get_time()
    # check if thrusters disabled
    if thrustEN:
        # joystick message
        if inversion == 4:
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
            if v_axis == 0:
                dhEnable = True
            else:
                dhEnable = False
            dh_enable_pub.publish(dhEnable)
    else:
        a_axis = 0
        l_axisLR = 0
        l_axisFB = 0
        v_axis = 0
        dhEnable = False

    # publish the vector values -> build up command vector message
    commandVectors = Twist()

    commandVectors.linear.x = l_axisLR
    commandVectors.linear.y = l_axisFB
    if dhEnable:
        commandVectors.linear.z = dh_eff
    else:
        commandVectors.linear.z = v_axis

    commandVectors.angular.x = a_axis

    # other angular axis for roll and pitch
    commandVectors.angular.y = roll_cmd_vel
    commandVectors.angular.z = 0

    vel_pub.publish(commandVectors)

# variable for monitoring the topic frequency so that a disconnect can be declared if the frequency drops below 1Hz
joyVerticalLastInput = 0.0

# what the node does when throttle publishes a new message
# joy "sensor_msgs/joy" message that is received when the joystick publishes a new message

def joyVerticalCallback(joy):
  global joyVerticalLastInput, useJoyVerticalAxis, v_axis, dhEnable, commandVectors
  # once copilot interface is created the params will be replaced with topics (inversion + sensitivity)
  joyVerticalLastInput = rospy.get_time()
  # check if thrusters disabled
  useJoyVerticalAxis = False
  if thrustEN:
    v_axis = joy.axes[verticalThrottleAxis] * v_scale * -1
    v_axis = expDrive(v_axis)

    # turn position-based depth hold on/off
    # dhEnableMsg = Bool()
    # if v_axis == 0:
        # dhEnable = True
        # dhEnableMsg.data = dhEnable
    # else:
        # dhEnable = False
        # dhEnableMsg.data = dhEnable

    # dh_enable_pub.publish(dhEnableMsg)

  # else:
      # v_axis = 0
      # dhEnable = False


  commandVectors = Twist()
  commandVectors.linear.x = l_axisLR
  commandVectors.linear.y = l_axisFB
  if dhEnable:
      commandVectors.linear.z = dh_eff
  else:
      commandVectors.linear.z = v_axis
  commandVectors.angular.x = a_axis

  # other angular axis for roll and pitch have phase 2 implementation
  commandVectors.angular.y = roll_cmd_vel
  commandVectors.angular.z = 0

  vel_pub.publish(commandVectors)


def joyWatchdogCB(data):
  global commandVectors, vel_pub, l_axisFB, l_axisLR, a_axis, useJoyVerticalAxis
  # checks the joystick
  if rospy.get_time() > joyHorizontalLastInput + 1.5:
      # ROS_ERROR("Joystick disconnection detected!")
      # publish the vector values for failsafe mode
      commandVectors = Twist() # Default message contains all zeros
      # Reset all the values to prevent feedback loop from throttle
      l_axisLR = 0
      l_axisFB = 0
      a_axis = 0
      if not useJoyVerticalAxis:
        # if the throttle is plugged in,then continue using the v_axis value
        commandVectors.linear.z = v_axis
        commandVectors.angular.y = roll_cmd_vel
        vel_pub.publish(commandVectors)

     # Check the throttle
if rospy.get_time() > joyVerticalLastInput + 1.5:
  # ROS_ERROR("Throttle disconnection detected!")
  useJoyVerticalAxis = True




# Handles copilot input: updates thrusters, enables sensitivity, and enables inversion.
# Callback to anything published by the dynamic reconfigure copilot page
# &config New copilot_interface param
# level The OR-ing of all the values that have changed in the copilot_interface param (not used yet)


def controlCallback(config, level):
    global thrustEN, l_scale, a_scale, v_scale, dhEnable, p_scalar, i_scalar, d_scalar
    thrustEN = config.thrusters

    l_scale = config.l_scale
    a_scale = config.a_scale
    v_scale = config.v_scale
    
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

    # PID depth hold publisher
    #dhMsg = PID()
    ##dhMsg.p_scalar = p_scalar
    #dhMsg.i_scalar = i_scalar
    #dhMsg.d_scalar = d_scalar
    #dh_pub.publish(dhMsg)
    
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
      depth.data =dhMostRecentDepth
      dh_setpoint_pub.publish(depth)



def dhControlEffortCallback(data): # no need for dhEnable check since PIDs won't publish control effort when disabled
  global dh_eff
  dh_eff = data.data


def rsControlEffortCallback(data):
  global roll_cmd_vel
  roll_cmd_vel = data.data



def main():
    global joy_sub1, joy_sub2, thruster_status_sub, sensitivity_sub, depth_hold_sub, dh_state_sub, dh_ctrl_eff_sub, dh_toggle_sub, rs_ctrl_eff_sub, inversion_pub, inversion_sub, vel_pub, camera_select, power_control, solenoid_control, sensitivity_pub, depth_hold_pub, thruster_status_pub, micro_status_pub, dh_cmd_vel_pub, dh_setpoint_pub, dh_enable_pub, lat_pid_pub, long_pid_pub, vert_pid_pub, roll_pid_pub, yaw_pid_pub
    joy_sub1 = rospy.Subscriber('joy/joy1', Joy, joyHorizontalCallback)
    joy_sub2 = rospy.Subscriber('joy/joy2', Joy, joyVerticalCallback)
    thruster_status_sub = rospy.Subscriber('rov/thruster_status', Bool,thrusterStatusCallback)
    sensitivity_sub = rospy.Subscriber('rov/sensitivity', rov_sensitivity, sensitivityCallback)
    #depth_hold_sub = rospy.Subscriber('depth_hold/pid_enable', PID, depthHoldCallback)
    dh_state_sub = rospy.Subscriber('odometry/filtered', Odometry, dhStateCallback)
    dh_ctrl_eff_sub = rospy.Subscriber('depth_hold/control_effort', Float64, dhControlEffortCallback)
    dh_toggle_sub = rospy.Subscriber('depth_hold/pid_enable', Bool, dhToggleCallback)
    rs_ctrl_eff_sub = rospy.Subscriber('roll_stabilization/control_effort', Float64, rsControlEffortCallback)
    inversion_sub = rospy.Subscriber('rov/inversion', UInt8, inversionCallback)

    vel_pub = rospy.Publisher('rov/cmd_vel', Twist, queue_size=1)
    inversion_pub = rospy.Publisher('rov/inversion', UInt8, queue_size=1)
    camera_select = rospy.Publisher('rov/camera_select', UInt8, queue_size=3)
    power_control = rospy.Publisher('tcu/main_relay', Bool, queue_size=3)
    solenoid_control = rospy.Publisher('tcu/main_solenoid', Bool, queue_size=3)
    sensitivity_pub = rospy.Publisher('rov/sensitivity', rov_sensitivity, queue_size=3)
    #depth_hold_pub = rospy.Publisher('depth_hold/pid_vals', PID, queue_size=3)
    thruster_status_pub = rospy.Publisher('rov/thruster_status', Bool, queue_size=3)
    dh_cmd_vel_pub = rospy.Publisher('rov/cmd_vel', Twist, queue_size=1)
    dh_setpoint_pub = rospy.Publisher('depth_hold/setpoint', Float64, queue_size=1)
    dh_enable_pub = rospy.Publisher('depth_hold/pid_enable', Bool, queue_size=1)

    # topics for PIDs
    lat_pid_pub = rospy.Publisher('/lat_motion/pid_enable', Bool, queue_size=1)
    long_pid_pub = rospy.Publisher('/long_motion/pid_enable', Bool, queue_size=1)
    vert_pid_pub = rospy.Publisher('/vert_motion/pid_enable', Bool, queue_size=1)
    roll_pid_pub = rospy.Publisher('/roll_motion/pid_enable', Bool, queue_size=1)
    yaw_pid_pub = rospy.Publisher('/yaw_motion/pid_enable', Bool, queue_size=1)




    # setup dynamic reconfigure
    server = Server(copilotControlParamsConfig, controlCallback)

    # create a ROS timer to call a callback that checks the joystick update rate (must be > 0.667Hz with ROS time)
    #joystickWatchdog = rospy.createTimer(rospy.Duration(1.5), joyWatchdogCB)

    # Enter the event loop
    rospy.spin()

if __name__  == "__main__":
    main()
