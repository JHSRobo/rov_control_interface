/**
* @author Michael Equi
* @version 0.1
* @date 8-11-2018
* @warning This file contains temporary implementations for cameras, tcu board control, and copilot interface.
* @mainpage The drive_control node
* @section intro_sec Introduction
* This code contains implementations for bilinear control, sensitivity, and 4-way inversion. The node subscribes to a joy topic and publishes rov/cmd_vel to PID algorithms and vector drive.
* @section compile_sec Compilation
* Compile using catkin_make in the ros_workspace directory.
*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>


#include <dynamic_reconfigure/server.h>
#include <copilot_interface/copilotControlParamsConfig.h>

#include <std_msgs/UInt8.h> //For camera Pub
#include <std_msgs/Bool.h>  //For tcu relay and solenoid controller Pub
#include <rov_control_interface/rov_sensitivity.h>
#include <std_msgs/Float64.h> //For pids
#include <std_msgs/Bool.h> //For pids
#include <nav_msgs/Odometry.h>

const int linearJoyAxisFBIndex(1); //!<forward-backward axis index in the joy topic array from the logitech Extreme 3D Pro
const int linearJoyAxisLRIndex(0); //!<left-right axis index in the joy topic array from the logitech Extreme 3D Pro
const int angularJoyAxisIndex(2);  //!<rotational axis index in the joy topic array from the logitech Extreme 3D Pro
const int verticalJoyAxisIndex(3); //!<vertical axis index in the joy topic array from the logitech Extreme 3D Pro

const int verticalThrottleAxis(2); //!<vertical axis index in the joy topic array from the Thrustmaster TWCS Throttle

double l_scale(0.5); //!< Holds a percent multiplier for sensitivity control. Default = 50%
double a_scale(0.5); //!< Holds a percent multiplier for sensitivity control. Default = 50%
double v_scale(0.5); //!< Holds a percent multiplier for sensitivity control. Default = 50%

double a_axis(0);   //!< Holds the value of the rotational/angular control axis
double l_axisLR(0); //!< Holds the value of the right-left linear control axis
double l_axisFB(0); //!< Holds the value of the front-back linear control axis
double v_axis(0);   //!< Holds the value of the vertical control axis
double dh_eff(0);   // holds depth hold PID control effort


bool thrustEN(false); //!<thrusters enabled (True = yes, False = default = no)

bool microEN(false);

//change this as a launch parameter in topside.launch
bool useJoyVerticalAxis(true); //!< Holds the state that determines wether the joysticks vertical input or the throttles vertical input gets used

//! inversion -> 1 Front, 2 Left, 3 Back, 4 Right, 5 Flipped
int inversion(0);

//! Variable for determining the bilinear threshold
const double bilinearRatio(1.5);

//! At what percent of the joysticks axis magnitude (-1 to 1) to apply the additional thrust
const double bilinearThreshold(1.0 / bilinearRatio);

//! Exponent for Drive Power Calculations
const double driveExp = 1.4;

ros::Publisher vel_pub; //!<publisher that publishes a Twist message containing 2 non-standard Vector3 data sets
ros::Subscriber joy_sub1; //!<subscriber to the logitech joystick
ros::Subscriber joy_sub2; //!<subscriber to the thrustmaster throttle

//Depth Hold
ros::Publisher dh_cmd_vel_pub; //publishes depth hold control effort to rov/cmd_vel
ros::Subscriber dh_ctrl_eff_sub; //subscribes to depth hold control effort
ros::Subscriber dh_state_sub; //keeps an up-to-date depth value for when depth hold is enabled, stops updating while depth hold is enabled
ros::Publisher dh_setpoint_pub; // publishes most recent depth to the depth_hold/setpoint
double dhMostRecentDepth(0); //holds depth for when depth hold needs to be enabled
ros::Subscriber dh_toggle_sub; //subscribes to depth_hold/pid_enable to update dhEnable for the state sub
bool dhEnable(false);
ros::Publisher dh_enable_pub; // turns depth hold on/off when joystick vert is 0/not 0

//Roll Stabilization
ros::Subscriber rs_ctrl_eff_sub; //subscribes to roll stab control effort
double roll_cmd_vel(0); // global to store roll_stab control effort for cmd_vel integration (there has to be a better way)

ros::Subscriber inversion_sub; //!<subscriber to inversion from copilot
ros::Subscriber sensitivity_sub; //!<subscriber to sensitivity from copilot
ros::Subscriber thruster_status_sub; //!<subscriber to thrusters enabled/disabled from copilot

ros::Publisher camera_select;    //!<Camera pub
ros::Publisher power_control;    //!<TCU relay controller
ros::Publisher solenoid_control; //!<TCU solenoid controller
ros::Publisher sensitivity_pub; //!<Publishes sensitivity from copilot
ros::Publisher thruster_status_pub; //!<Publishes thruster status from copilot
ros::Publisher micro_status_pub;
ros::Publisher lat_pid_pub;
ros::Publisher long_pid_pub;
ros::Publisher vert_pid_pub;
ros::Publisher roll_pid_pub;
ros::Publisher pitch_pid_pub;
ros::Publisher yaw_pid_pub;

/**
* @brief Controls variable joystick sensitivity. Small movements that use a small percent of the maximum control vector magnitude have a lower sensitivity than larger movements with the joystick.
* @param[in,out] axis Takes in a reference to the axis (a_axis, l_axisLR/FB, v_axis) from -1 to 1
*/
void expDrive (double &axis, const double &driveExp)
{
    axis = copysign((pow(fabs(axis), driveExp)), axis); // Copies
}

//! variable for monitoring the topic frequency so that a diconnect can be declared if the frequency drops below 1Hz
double joyHorizontalLastInput(0.0);
/**
* @breif What the node does when joystick publishes a new message
* @param[in] joy "sensor_msgs/Joy" message that is recieved when the joystick publsihes a new message
*/
void joyHorizontalCallback(const sensor_msgs::Joy::ConstPtr& joy){
    //once copilot interface is created the params will be replaced with topics (inversion + sensitivity)
    joyHorizontalLastInput = ros::Time::now().toSec();
    //check if thrusters disabled
    if (thrustEN) {
        //joystick message
        //float32[] axes          the axes measurements from a joystick
        //int32[] buttons         the buttons measurements from a joystick

        //store axes variables and handle 4 cases of inversion
        if (inversion == 5)
            a_axis = joy->axes[angularJoyAxisIndex] * a_scale; // Don't change sign when using the backup camera
        else
            a_axis = joy->axes[angularJoyAxisIndex] * a_scale * -1; //changing sign makes rotate right positive

        //NOTE: right and rotate right are negative on the joystick's LR axis
        //multiple LR axis by -1 in base position (front-front, etc.)to make right positive

        switch (inversion){
        case 1 : //right side is front
            l_axisFB = joy->axes[linearJoyAxisLRIndex] * l_scale * -1;
            l_axisLR = joy->axes[linearJoyAxisFBIndex] * l_scale;
            break;
        case 2 : //back side is front
            l_axisLR = joy->axes[linearJoyAxisLRIndex] * l_scale;
            l_axisFB = joy->axes[linearJoyAxisFBIndex] * l_scale * -1;
            break;
        case 3 : //left side is front
            l_axisFB = joy->axes[linearJoyAxisLRIndex] * l_scale;
            l_axisLR = joy->axes[linearJoyAxisFBIndex] * l_scale * -1;
            break;
        default: //front side is front
            l_axisLR = joy->axes[linearJoyAxisLRIndex] * l_scale * -1;
            l_axisFB = joy->axes[linearJoyAxisFBIndex] * l_scale;
            break;
        }


        //apply the exponetial ratio on all axis
        expDrive(a_axis, driveExp);
        expDrive(l_axisLR, driveExp);
        expDrive(l_axisFB, driveExp);
        if(useJoyVerticalAxis){
          v_axis = joy->axes[verticalJoyAxisIndex] * v_scale * -1;
          expDrive(v_axis, driveExp);

          // turn position-based depth hold on/off
          std_msgs::Bool dhEnableMsg;
          if (v_axis == 0) {
            dhEnable = true;
            dhEnableMsg.data = dhEnable;
          }
          else {
            dhEnable = false;
            dhEnableMsg.data = dhEnable;
          }
          dh_enable_pub.publish(dhEnableMsg);
        }



    } else {
        a_axis = 0;
        l_axisLR = 0;
        l_axisFB = 0;
        v_axis = 0;
        dhEnable = false;
    }

    //publish the vector values -> build up command vector message
    geometry_msgs::Twist commandVectors;

    commandVectors.linear.x = l_axisLR;
    commandVectors.linear.y = l_axisFB;
    if (dhEnable){
      commandVectors.linear.z = dh_eff;
    }
    else {
      commandVectors.linear.z = v_axis; //linear z is for vertical strength
    }
    commandVectors.angular.x = a_axis;

    //other angular axis for roll and pitch have phase 2 implementation
    commandVectors.angular.y = roll_cmd_vel;
    commandVectors.angular.z = 0;

    vel_pub.publish(commandVectors);
}

//! variable for monitoring the topic frequency so that a diconnect can be declared if the frequency drops below 1Hz
double joyVerticalLastInput(0.0);
/**
* @breif What the node does when throttle publishes a new message
* @param[in] joy "sensor_msgs/Joy" message that is recieved when the joystick publsihes a new message
*/
void joyVerticalCallback(const sensor_msgs::Joy::ConstPtr& joy){
      //once copilot interface is created the params will be replaced with topics (inversion + sensitivity)
      joyVerticalLastInput = ros::Time::now().toSec();
      //check if thrusters disabled
      useJoyVerticalAxis = false;
      if (thrustEN) {
          //joystick message
          //float32[] axes          the axes measurements from a joystick
          //int32[] buttons         the buttons measurements from a joystick
          //store axes variables and handle 4 cases of inversion
          v_axis = joy->axes[verticalThrottleAxis] * v_scale * -1;
          expDrive(v_axis, driveExp);

          // turn position-based depth hold on/off
          std_msgs::Bool dhEnableMsg;
          if (v_axis == 0) {
            dhEnable = true;
            dhEnableMsg.data = dhEnable;
          }
          else {
            dhEnable = false;
            dhEnableMsg.data = dhEnable;
          }
          dh_enable_pub.publish(dhEnableMsg);

      } else {
          v_axis = 0;
          dhEnable = false;
      }

      //publish the vector values -> build up command vector message
      geometry_msgs::Twist commandVectors;

      commandVectors.linear.x = l_axisLR;
      commandVectors.linear.y = l_axisFB;
      if (dhEnable){
        commandVectors.linear.z = dh_eff;
      }
      else {
        commandVectors.linear.z = v_axis; //linear z is for vertical strength
      }
      commandVectors.angular.x = a_axis;

      //other angular axis for roll and pitch have phase 2 implementation
      commandVectors.angular.y = roll_cmd_vel;
      commandVectors.angular.z = 0;

      vel_pub.publish(commandVectors);
}

void joyWatchdogCB(const ros::TimerEvent&){
  //Check the joystick
  if(ros::Time::now().toSec() > joyHorizontalLastInput + 1.5){
//     ROS_ERROR("Joystick disconnection detected!");
    //publish the vector values for failsafe mode
    geometry_msgs::Twist commandVectors; //Default message contains all zeros
    // Reset all the values to prevent feedback loop from throttle
    l_axisLR = 0;
    l_axisFB = 0;
    a_axis = 0;
    if(!useJoyVerticalAxis){
      //if the throttle is plugged in, then continue using the v_axis value
      commandVectors.linear.z = v_axis;
    }
    commandVectors.angular.y = roll_cmd_vel;
    vel_pub.publish(commandVectors);
  }

  //Check the throttle
  if(ros::Time::now().toSec() > joyVerticalLastInput + 1.5){
    // ROS_ERROR("Throttle disconnection detected!");
    useJoyVerticalAxis = true;
  }
}


/**
* @breif Handles copilot input: updates thrusters, enables sensitivity, and enables inversion.
* Callback to anything published by the dynamic reconfigure copilot page
* @param[in] &config New copilot_interface param
* @param[in] level The OR-ing of all the values that have changed in the copilot_interface param (not used yet)
*/
void controlCallback(copilot_interface::copilotControlParamsConfig &config, uint32_t level) {
    thrustEN = config.thrusters;
    microEN = config.microrov;

    l_scale = config.l_scale;
    a_scale = config.a_scale;
    v_scale = config.v_scale;


    // Sensitivty Publisher
     rov_control_interface::rov_sensitivity sensitivityMsg;
     sensitivityMsg.l_scale = l_scale;
     sensitivityMsg.a_scale = a_scale;
     sensitivityMsg.v_scale = v_scale;
     sensitivity_pub.publish(sensitivityMsg);

    // Thrusters Enabled Publisher
    std_msgs::Bool thrusterStatusMsg;
    thrusterStatusMsg.data = thrustEN;
    thruster_status_pub.publish(thrusterStatusMsg);

    // Micro ROV Enabled Publisher
    std_msgs::Bool microStatusMsg;
    microStatusMsg.data = microEN;
    micro_status_pub.publish(microStatusMsg);

}

/**
* @breif What the node does when copilot inversion setting publishes a new message
* @param[in] joy "sensor_msgs/Joy" message that is recieved when the joystick publsihes a new message
*/
void inversionCallback(const std_msgs::UInt8::ConstPtr& data) {
    inversion = data->data;
    std::cout << data->data;
}

/**
* @breif What the node does when copilot sensitivity setting publishes a new message
* @param[in] "rov_control_interface/rov_sensitivity." message that is recieved when the sensitivty setting is changed
*/
 void sensitivityCallback(const rov_control_interface::rov_sensitivity::ConstPtr& data) {
   l_scale = data->l_scale;
   a_scale = data->a_scale;
   v_scale = data->v_scale;
 }

/**
* @breif What the node does when thruster status topic publishes a new message
* @param[in] "std_msgs/Bool" message that is recieved when the topic publsihes a new message
*/
void thrusterStatusCallback(const std_msgs::Bool::ConstPtr& data) {
  thrustEN = data->data;
  ROS_INFO_STREAM(thrustEN);
}

void microStatusCallback(const std_msgs::Bool::ConstPtr& data) {
  microEN = data->data;
  ROS_INFO_STREAM(microEN);
}

void dhToggleCallback(const std_msgs::Bool::ConstPtr& data) {
  dhEnable = data->data;
}

void dhStateCallback(const nav_msgs::Odometry::ConstPtr& data) {
  if (!dhEnable) { //only update depth if depth hold is disabled (dhEnable == false)
    dhMostRecentDepth = data->pose.pose.position.z * -1;
    std_msgs::Float64 depth;
    depth.data = dhMostRecentDepth;
    dh_setpoint_pub.publish(depth);
  }
}

void dhControlEffortCallback(const std_msgs::Float64::ConstPtr& data) { // no need for dhEnable check since pids won't publish control effort when disabled
  dh_eff = data->data;
}

void rsControlEffortCallback(const std_msgs::Float64::ConstPtr& data){
  roll_cmd_vel = data->data;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "drive_control");

    ros::NodeHandle n;

    //setup publisher and subscriber
    joy_sub1 = n.subscribe<sensor_msgs::Joy>("joy/joy1", 2, &joyHorizontalCallback);
    joy_sub2 = n.subscribe<sensor_msgs::Joy>("joy/joy2", 2, &joyVerticalCallback);
    thruster_status_sub = n.subscribe<std_msgs::Bool>("rov/thruster_status", 1, &thrusterStatusCallback);
    sensitivity_sub = n.subscribe<rov_control_interface::rov_sensitivity>("rov/sensitivity", 3, &sensitivityCallback);
    dh_state_sub = n.subscribe<nav_msgs::Odometry>("odometry/filtered", 1, &dhStateCallback);
    dh_ctrl_eff_sub = n.subscribe<std_msgs::Float64>("depth_hold/control_effort", 1, &dhControlEffortCallback);
    dh_toggle_sub = n.subscribe<std_msgs::Bool>("depth_hold/pid_enable", 1, &dhToggleCallback);
    rs_ctrl_eff_sub = n.subscribe<std_msgs::Float64>("roll_stabilization/control_effort", 1, &rsControlEffortCallback);
    inversion_sub = n.subscribe<std_msgs::UInt8>("rov/inversion", 1, &inversionCallback);

    vel_pub = n.advertise<geometry_msgs::Twist>("rov/cmd_vel", 1);
    camera_select = n.advertise<std_msgs::UInt8>("rov/camera_select", 3);       //Camera pub
    power_control = n.advertise<std_msgs::Bool>("tcu/main_relay", 3);       //Relay pub
    solenoid_control = n.advertise<std_msgs::Bool>("tcu/main_solenoid", 3); //Solenoid pub
    sensitivity_pub = n.advertise<rov_control_interface::rov_sensitivity>("rov/sensitivity", 3);
    thruster_status_pub = n.advertise<std_msgs::Bool>("rov/thruster_status", 3);
    micro_status_pub = n.advertise<std_msgs::Bool>("micro/enable", 3);
    dh_cmd_vel_pub = n.advertise<geometry_msgs::Twist>("rov/cmd_vel", 1);
    dh_setpoint_pub = n.advertise<std_msgs::Float64>("depth_hold/setpoint", 1);
    dh_enable_pub = n.advertise<std_msgs::Bool>("depth_hold/pid_enable", 1);

    //topics for PIDs
    lat_pid_pub = n.advertise<std_msgs::Bool>("/lat_motion/pid_enable", 1);
    long_pid_pub = n.advertise<std_msgs::Bool>("/long_motion/pid_enable", 1);
    vert_pid_pub = n.advertise<std_msgs::Bool>("/vert_motion/pid_enable", 1);
    roll_pid_pub = n.advertise<std_msgs::Bool>("/roll_motion/pid_enable", 1);
//    pitch_pid_pub = n.advertise<std_msgs::Bool>("/pitch_motion/pid_enable", 1);
    yaw_pid_pub = n.advertise<std_msgs::Bool>("/yaw_motion/pid_enable", 1);




    //setup dynamic reconfigure
    dynamic_reconfigure::Server<copilot_interface::copilotControlParamsConfig> server;
    dynamic_reconfigure::Server<copilot_interface::copilotControlParamsConfig>::CallbackType f;

    f = boost::bind(&controlCallback, _1, _2);
    server.setCallback(f);

    //create a ROS timer to call a callback that checks the joystick update rate (must be > 0.667Hz with ROS time)
    ros::Timer joystickWatchdog = n.createTimer(ros::Duration(1.5), joyWatchdogCB);

    //Enter the event loop
    ros::spin();

    return 0;
}
