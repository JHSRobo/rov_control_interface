#!/usr/bin/env python
# author Adon Sharp | v0.2 | 1-15-2022
# author Alex Bertran | v1.0 | 10-29-2022

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import UInt8  # For camera  pub
from math import copysign
from dynamic_reconfigure.server import Server
from copilot_interface.cfg import copilotControlParamsConfig
from rov_control_interface.msg import thrusterPercents

rospy.init_node("drive_control")

sensitivity = {"linear": 0.5, "angular": 0.25, "vertical": 0.5} # Holds a percent multiplier for ROV sensitivity

thrustEN = False  # thrusters enabled (True = yes, False = default = no

#The vector that gets edited by the callbacks and then published
joyVector = Twist()

# Multiplies the value of the axis by an exponent (1.4)
# Uses copysign to make sure that raising axis to an even power can still return a negative number
def expDrive(axis):
  axis = copysign(abs(axis) ** 1.4, axis)
  return axis

# Takes in vectors and translates them to thrusterPercents
  # linearX is the left-right joystick axis
  # linearY is the front-back joystick axis
  # linearZ is the throttle
  # angularX is the rotational joystick axis
def translateVectors(linearX, linearY, linearZ, angularX):
  #Check to make sure values are appropriate
  if abs(linearX) > 1 or abs(linearY) > 1 or abs(linearZ) > 1 or abs(angularX) > 1: # The values for max are subject to change
    rospy.logerr("Vectors outside of range")
  
  # Motor Calculations
  # IMPORTANT TO UNDERSTAND THIS: https://drive.google.com/file/d/11VF0o0OYVaFGKFvbYtnrmOS0e6yM6IxH/view
  # TOTALLY SUBJECT TO CHANGE IN EVENT OF THRUSTER REARRANGEMENT
  # T is the name of the dictionary where we store our temp thruster vals. This is a dict and not a list so we can start at index 1.
  # This is advantageouss because it means T[1] lines up with Thruster #1 on the ROV.
  T = {1 : linearX + linearY + angularX, 2 : -linearX + linearY - angularX, 3 : -linearX - linearY + angularX,
       4 : linearX - linearY - angularX, 5 : linearZ, 6 : linearZ}
  T[1] = linearX + linearY + angularX
  T[2] = -linearX + linearY - angularX
  T[3] = -linearX - linearY + angularX
  T[4] = linearX - linearY - angularX
  T[5] = linearZ
  T[6] = linearZ
  
  # Do a little math to normalize the values
  maxMotor = max(abs(T[1]), abs(T[2]), abs(T[3]), abs(T[4]), abs(T[5]), abs(T[6]))
  maxInput = max(abs(linearX), abs(linearY), abs(linearZ), abs(angularX))
  if maxMotor == 0:
    maxMotor = 1

  for key in T:
    T[key] *= maxInput / maxMotor
  
  # Load up the thrusterVals message with our calculated values
  thrusterVals = thrusterPercents() # The amount we multiply each value by is just what we had in the original code. Subject to change

  thrusterVals.t1 = T[1] * 1000
  thrusterVals.t2 = T[2] * 1000
  thrusterVals.t3 = T[3] * 1000
  thrusterVals.t4 = T[4] * 1000
  thrusterVals.t5 = T[5] * 1000
  thrusterVals.t6 = T[6] * 1000
  
  return thrusterVals


def joyHorizontalCallback(joy):
    global camera_select, joyVector, sensitivity, thrustEN
    
    # Bodge code for camera switching. Move to joystick program later on.
    if joy.buttons[2]:
        camera_select.publish(1)
    elif joy.buttons[3]:
        camera_select.publish(2)
    elif joy.buttons[4]:
        camera_select.publish(3)
    elif joy.buttons[5]:
        camera_select.publish(4)

    # If thrusters enabled, map the joystick inputs to the joyVector
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

    velPub.publish(joyVector)

# Callback that runs whenever the throttle sends an update
def joyVerticalCallback(joy):
  global thrustEN, joyVector, sensitivity

  # check if thrusters disabled
  if thrustEN:
    v_axis = joy.axes[2] * sensitivity['vertical'] * -1

    v_axis = expDrive(v_axis)
  else:
    v_axis = 0

  joyVector.linear.z = v_axis
  velPub.publish(joyVector)

# Handles copilot input: updates thrusters, edits sensitivity
# Callback to anything published by the dynamic reconfigure copilot page

def controlCallback(config, level):
    global thrustEN, sensitivity
    
    thrustEN = config.thrusters
    sensitivity['linear'] = config.l_scale
    sensitivity['angular'] = config.a_scale
    sensitivity['vertical'] = config.v_scale
    
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

    return config

def ROS_INFO_STREAM(thrustEN):
    pass
  
def thrusterStatusCallback(data):
    global thrustEN
    thrustEN = data.data
    ROS_INFO_STREAM(thrustEN)
  
def main():
    global horizJoySub, vertJoySub, velPub, camera_select
    horizJoySub = rospy.Subscriber('joy/joy1', Joy, joyHorizontalCallback)
    vertJoySub = rospy.Subscriber('joy/joy2', Joy, joyVerticalCallback)
    thruster_status_sub = rospy.Subscriber('rov/thruster_status', Bool, thrusterStatusCallback)

    velPub = rospy.Publisher('thrusters', Twist, queue_size=1)
    camera_select = rospy.Publisher('rov/camera_select', UInt8, queue_size=3)
    thruster_status_pub = rospy.Publisher('rov/thruster_status', Bool, queue_size=3)

    # setup dynamic reconfigure
    server = Server(copilotControlParamsConfig, controlCallback)

    # Enter the event loop
    rospy.spin()

if __name__  == "__main__":
    main()
