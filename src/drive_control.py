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

thrustEN = False  # thrusters enabled (True = yes, False = default = no

#The vector that gets edited by the callbacks and then published
joyVector = Twist()

# Multiplies the value of the axis by an exponent (1.4)
# Uses copysign to make sure that raising axis to an even power can still return a negative number
def expDrive(axis):
  axis = copysign(abs(axis) ** 1.4, axis)
  return axis

def joyHorizontalCallback(joy):
    global camera_select, joyVector
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
        l_axisLR = joy.axes[0] * sensitivity['linear'] * -1
        l_axisFB = joy.axes[1] * sensitivity['linear']
        a_axis = joy.axes[2] * sensitivity['angular'] * -1 

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
  global v_axis, dhEnable, joyVector

  # check if thrusters disabled
  if thrustEN:
    v_axis = joy.axes[2] * sensitivity['vertical'] * -1
    v_axis = expDrive(v_axis)
  else:
    v_axis = 0

  joyVector.linear.z = v_axis
  vel_pub.publish(joyVector)

# Handles copilot input: updates thrusters, edits sensitivity
# Callback to anything published by the dynamic reconfigure copilot page

def controlCallback(config, level):
    global thrustEN, sensitivity
    
    thrustEN = config.thrusters

    sensitivity['linear'] = config.l_scale
    sensitivity['angular'] = config.a_scale
    sensitivity['vertical'] = config.v_scale

    return config

def main():
    global horizJoySub, vertJoySub, vel_pub, camera_select
    horizJoySub = rospy.Subscriber('joy/joy1', Joy, joyHorizontalCallback)
    vertJoySub = rospy.Subscriber('joy/joy2', Joy, joyVerticalCallback)

    vel_pub = rospy.Publisher('rov/cmd_vel', Twist, queue_size=1)
    camera_select = rospy.Publisher('rov/camera_select', UInt8, queue_size=3)

    # setup dynamic reconfigure
    server = Server(copilotControlParamsConfig, controlCallback)

    # Enter the event loop
    rospy.spin()

if __name__  == "__main__":
    main()
