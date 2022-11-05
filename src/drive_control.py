#!/usr/bin/env python
# author Adon Sharp | v0.2 | 1-15-2022
# author Alex Bertran | v1.0 | 10-29-2022

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

rospy.init_node("drive_control")
verticalJoyAxisIndex = 3  # vertical axis index in the joy topic array from the logitech Extreme 3D Pro

sensitivity = {"linear": 0.5, "angular": 0.25, "vertical": 0.5} # Holds a percent multiplier for ROV sensitivity

a_axis = 0  # Holds the value of the rotational/angular control axis
l_axisLR = 0  # Holds the value of the right-left linear control axis
l_axisFB = 0  # Holds the value of the front-back linear control axis
v_axis = 0  # Holds the value of the vertical control axis


thrustEN = False  # thrusters enabled (True = yes, False = default = no
dhEnable = False  # for depth hold, TEMPORARY PLACEHOLDER TO PREVENT ERRORS

<<<<<<< Updated upstream
useJoyVerticalAxis = True  # Holds the state that determines whether the joysticks vertical input of the throttles vertical input gets used

# inversion -> 1 Front, 2 Left, 3 Back, 4 Right, 5 Flipped; used when switching perspectives on ROV
inversion = 0

# Exponent for Drive Power Calculations
driveExp = 1.4

roll_cmd_vel = 0  # global to store roll_stab control effort for cmd_vel integration (there has to be a better way)

# Vectors in 2D or 1D
vertJoyVector = Twist() # DELETE THIS LATER

=======
>>>>>>> Stashed changes
#The vector that gets edited by the callbacks and then published
joyVector = Twist()

# Multiplies the value of the axis by an exponent (1.4)
# Uses copysign to make sure that raising axis to an even power can still return a negative number
def expDrive(axis):
  axis = copysign(abs(axis) ** 1.4, axis)
  return axis

# variable for monitoring the topic frequency so that a disconnect can be declared if the frequency drops below 1Hz
joyHorizontalLastInput = 0.0


def joyHorizontalCallback(joy):
    global joyHorizontalLastInput, a_axis, l_axisLR, l_axisFB, camera_select, joyVector
    #Bodge code for camera switching. Move to joystick program later on.
    
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
        # Multiply LR axis by -1 in base position (front-front, etc.)to make right positive
        # NOTE: right and rotate right are negative on the joystick's LR axis
<<<<<<< Updated upstream
        a_axis = joy.axes[angularJoyAxisIndex] * a_scale * -1

        l_axisLR = joy.axes[linearJoyAxisLRIndex] * l_scale * -1 
        l_axisFB = joy.axes[linearJoyAxisFBIndex] * l_scale
=======
        l_axisLR = joy.axes[0] * sensitivity['linear'] * -1
        l_axisFB = joy.axes[1] * sensitivity['linear']
        a_axis = joy.axes[2] * sensitivity['angular'] * -1 
>>>>>>> Stashed changes

        # Apply the exponential ratio on all axis
        a_axis = expDrive(a_axis)
        l_axisLR = expDrive(l_axisLR)
        l_axisFB = expDrive(l_axisFB)

    else:
        a_axis = 0
        l_axisLR = 0
        l_axisFB = 0

    # Add the created vectors to the joyVector
    joyVector.linear.x = l_axisLR
    joyVector.linear.y = l_axisFB
    joyVector.angular.x = a_axis

    vel_pub.publish(joyVector)

# what the node does when throttle publishes a new message
# joy "sensor_msgs/joy" message that is received when the joystick publishes a new message

def joyVerticalCallback(joy):
  global joyVerticalLastInput, useJoyVerticalAxis, v_axis, dhEnable, joyVector
  # once copilot interface is created the params will be replaced with topics (inversion + sensitivity)
  joyVerticalLastInput = rospy.get_time()
  # check if thrusters disabled
<<<<<<< Updated upstream
  useJoyVerticalAxis = False
  if thrustEN and dhEnable == False:
    v_axis = joy.axes[verticalThrottleAxis] * v_scale * -1
=======
  if thrustEN:
    v_axis = joy.axes[2] * sensitivity['vertical'] * -1
>>>>>>> Stashed changes
    v_axis = expDrive(v_axis)



  #if dhEnable:
      #joyVertVector.linear.z = dh_eff
  #else:
  joyVector.linear.z = v_axis
  vel_pub.publish(joyVector)

<<<<<<< Updated upstream

# Handles copilot input: updates thrusters, enables sensitivity, and enables inversion.
=======
# Handles copilot input: updates thrusters, edits sensitivity
>>>>>>> Stashed changes
# Callback to anything published by the dynamic reconfigure copilot page
# &config New copilot_interface param
# level The OR-ing of all the values that have changed in the copilot_interface param (not used yet)


def controlCallback(config, level):
<<<<<<< Updated upstream
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

=======
    global thrustEN, sensitivity
    
    thrustEN = config.thrusters

    sensitivity['linear'] = config.l_scale
    sensitivity['angular'] = config.a_scale
    sensitivity['vertical'] = config.v_scale
>>>>>>> Stashed changes

    return config

def main():
<<<<<<< Updated upstream
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
=======
    global horizJoySub, vertJoySub, vel_pub, camera_select
    horizJoySub = rospy.Subscriber('joy/joy1', Joy, joyHorizontalCallback)
    vertJoySub = rospy.Subscriber('joy/joy2', Joy, joyVerticalCallback)
>>>>>>> Stashed changes

    vel_pub = rospy.Publisher('rov/cmd_vel', Twist, queue_size=1)
    camera_select = rospy.Publisher('rov/camera_select', UInt8, queue_size=3)

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
