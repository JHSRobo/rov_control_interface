#!/usr/bin/env python
# author Alex Bertran | v0.0 | 11-11-2022
# Current imports are all those from drive_control.py and some may not be necessary
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

from std_msgs.msg import UInt8  # For camera  pub
from std_msgs.msg import Bool  # For TCU relay and solenoid controller pub and for pids
from std_msgs.msg import Float64  # For pids
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry

from math import copysign
from dynamic_reconfigure.server import Server
from copilot_interface.cfg import copilotControlParamsConfig

rospy.init_node("autonomous_control")

thrustEN = False
dhEnable = False

def controlCallback(config, level):
  global thrustEN, dhEnable, p_scalar, i_scalar, d_scalar
  
  thrustEN = config.thrusters
  p_scalar = config.p_scalar
  i_scalar = config.i_scalar
  d_scalar = config.d_scalar
  dhEnable = config.dh_enable
  
  return config

def ROS_INFO_STREAM(thrustEN):
  pass

def thrusterStatusCallback(data):
  global thrustEN
  thrustEN = data.data
  ROS_INFO_STREAM(thrustEN)

def depthHoldCallback(data):
  global p_scalar, i_scalar, d_scalar
    
  p_scalar = data.p_scalar
  i_scalar = data.i_scalar
  d_scalar = data.d_scalar

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
  
def change_depth_callback(depth):
  global dhEnable, thrustEN, test_pub
  #THIS FOLLOWING CODE IS INTENTIONALLY CRAP. REMOVE WHEN POSSIBLE
  test_pub = rospy.Publisher('/rov/thruster_testing', Int32, queue_size=1)

  rospy.loginfo("depth recieved")  
  if thrustEN and dhEnable:
    currentDepth = abs((depth.data - 198.3) / (893.04 / 149))
    test_pub.publish(currentDepth)
  
def main():
  global thruster_status_sub, depth_hold_sub, dh_state_sub, dh_ctrl_eff_sub, dh_toggle_sub, depth_sub
  
  #test_pub = rospy.Publisher('/rov/thruster_testing', Int32, queue_size=1)
  thruster_status_sub = rospy.Subscriber('rov/thruster_status', Bool,thrusterStatusCallback)
  #depth_hold_sub = rospy.Subscriber('depth_hold/pid_enable', PID, depthHoldCallback)
  dh_state_sub = rospy.Subscriber('odometry/filtered', Odometry, dhStateCallback)
  dh_ctrl_eff_sub = rospy.Subscriber('depth_hold/control_effort', Float64, dhControlEffortCallback)
  dh_toggle_sub = rospy.Subscriber('depth_hold/pid_enable', Bool, dhToggleCallback)
  depth_sub = rospy.Subscriber('rov/depth_sensor', Float32, change_depth_callback)
  
  server = Server(copilotControlParamsConfig, controlCallback)
  
  rospy.spin()

if __name__  == "__main__":
    main()
