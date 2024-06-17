/**
Software License Agreement (BSD)

\authors   Mike Purvis <mpurvis@clearpathrobotics.com>
\copyright Copyright (c) 2014, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "teleop_twist_joy/teleop_twist_joy.h"

#include <map>
#include <string>


namespace teleop_twist_joy
{

/**
 * Internal members of class. This is the pimpl idiom, and allows more flexibility in adding
 * parameters later without breaking ABI compatibility, for robots which link TeleopTwistJoy
 * directly into base nodes.
 */
struct TeleopTwistJoy::Impl
{
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void sendCmdVelMsg(const sensor_msgs::Joy::ConstPtr& joy_msg, const std::string& which_map);
  double getVal(const sensor_msgs::Joy::ConstPtr& joy_msg, const std::map<std::string, int>& axis_map, const std::map<std::string,
                double>& scale_map, const std::string& fieldname, const std::map<std::string, double>& turbo_scale_map);

  ros::Subscriber joy_sub;
  ros::Publisher cmd_vel_pub;

  double k_expo;
  int enable_button;
  int enable_turbo_button;

  float base_width;
  int enable_track_control_button;
  float deadzone;
  int track_on;
  int timer;

  std::map<std::string, int> axis_linear_map;
  std::map< std::string, std::map<std::string, double> > scale_linear_map;

  std::map<std::string, int> axis_angular_map;
  std::map< std::string, std::map<std::string, double> > scale_angular_map;

  bool sent_disable_msg;
};

/**
 * Constructs TeleopTwistJoy.
 * \param nh NodeHandle to use for setting up the publisher and subscriber.
 * \param nh_param NodeHandle to use for searching for configuration parameters.
 */
TeleopTwistJoy::TeleopTwistJoy(ros::NodeHandle* nh, ros::NodeHandle* nh_param)
{
  pimpl_ = new Impl;

  pimpl_->cmd_vel_pub = nh->advertise<geometry_msgs::Twist>("hri_cmd_vel", 1, true);
  pimpl_->joy_sub = nh->subscribe<sensor_msgs::Joy>("hri_joy", 1, &TeleopTwistJoy::Impl::joyCallback, pimpl_);
  pimpl_->track_on = 1;
  pimpl_->timer = 0;
  nh_param->param<int>("enable_button", pimpl_->enable_button, 0);
  nh_param->param<int>("enable_turbo_button", pimpl_->enable_turbo_button, -1);
  nh_param->param<int>("enable_track_control_button", pimpl_->enable_track_control_button, 6);
  nh_param->param<double>("k_expo", pimpl_->k_expo, 1);
  nh_param->param<float>("base_width", pimpl_->base_width, 1.1652);
  nh_param->param<float>("deadzone", pimpl_->deadzone, 0);

  if (nh_param->getParam("axis_linear", pimpl_->axis_linear_map))
  {
    nh_param->getParam("scale_linear", pimpl_->scale_linear_map["normal"]);
    nh_param->getParam("scale_linear_turbo", pimpl_->scale_linear_map["turbo"]);
  }
  else
  {
    nh_param->param<int>("axis_linear", pimpl_->axis_linear_map["x"], 1);
    nh_param->param<double>("scale_linear", pimpl_->scale_linear_map["normal"]["x"], 0.5);
    nh_param->param<double>("scale_linear_turbo", pimpl_->scale_linear_map["turbo"]["x"], 1.0);
  }

  if (nh_param->getParam("axis_angular", pimpl_->axis_angular_map))
  {
    nh_param->getParam("scale_angular", pimpl_->scale_angular_map["normal"]);
    nh_param->getParam("scale_angular_turbo", pimpl_->scale_angular_map["turbo"]);
  }
  else
  {
    nh_param->param<int>("axis_angular", pimpl_->axis_angular_map["yaw"], 0);
    nh_param->param<double>("scale_angular", pimpl_->scale_angular_map["normal"]["yaw"], 0.5);
    nh_param->param<double>("scale_angular_turbo", pimpl_->scale_angular_map["turbo"]["yaw"], 1.0);
  }

  ROS_INFO_NAMED("TeleopTwistJoy", "Teleop enable button %i.", pimpl_->enable_button);
  ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0, "TeleopTwistJoy",
      "Turbo on button %i.", pimpl_->enable_turbo_button);

  for (std::map<std::string, int>::iterator it = pimpl_->axis_linear_map.begin();
      it != pimpl_->axis_linear_map.end(); ++it)
  {
    ROS_INFO_NAMED("TeleopTwistJoy", "Linear axis %s on %i at scale %f.",
    it->first.c_str(), it->second, pimpl_->scale_linear_map["normal"][it->first]);
    ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0, "TeleopTwistJoy",
        "Turbo for linear axis %s is scale %f.", it->first.c_str(), pimpl_->scale_linear_map["turbo"][it->first]);
  }

  for (std::map<std::string, int>::iterator it = pimpl_->axis_angular_map.begin();
      it != pimpl_->axis_angular_map.end(); ++it)
  {
    ROS_INFO_NAMED("TeleopTwistJoy", "Angular axis %s on %i at scale %f.",
    it->first.c_str(), it->second, pimpl_->scale_angular_map["normal"][it->first]);
    ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0, "TeleopTwistJoy",
        "Turbo for angular axis %s is scale %f.", it->first.c_str(), pimpl_->scale_angular_map["turbo"][it->first]);
  }

  pimpl_->sent_disable_msg = false;
}

    double TeleopTwistJoy::Impl::getVal(const sensor_msgs::Joy::ConstPtr& joy_msg, const std::map<std::string, int>& axis_map,
                                        const std::map<std::string, double>& scale_map, const std::string& fieldname,
                                        const std::map<std::string, double>& turbo_scale_map)
{
  if (axis_map.find(fieldname) == axis_map.end() ||
      scale_map.find(fieldname) == scale_map.end() ||
      joy_msg->axes.size() <= axis_map.at(fieldname))
  {
    return 0.0;
  }

  double turbo_scale = (1.0 + fabs(joy_msg->axes[enable_turbo_button] * (turbo_scale_map.at(fieldname) - scale_map.at(fieldname))));
  return (turbo_scale * (pow(joy_msg->axes[axis_map.at(fieldname)], 3) * (k_expo - 1) + joy_msg->axes[axis_map.at(fieldname)] / k_expo * scale_map.at(fieldname)));
//  return joy_msg->axes[axis_map.at(fieldname)] * scale_map.at(fieldname);
}

//init the motionconverter func that whe are using later in the class
void TeleopTwistJoy::Impl::sendCmdVelMsg(const sensor_msgs::Joy::ConstPtr& joy_msg,
                                         const std::string& which_map)
{
  // Initializes with zeros by default.
  geometry_msgs::Twist cmd_vel_msg;
if ( track_on == 1)
{
  cmd_vel_msg.linear.x = getVal(joy_msg, axis_linear_map, scale_linear_map[which_map], "x", scale_linear_map["turbo"]);
  cmd_vel_msg.linear.y = getVal(joy_msg, axis_linear_map, scale_linear_map[which_map], "y", scale_linear_map["turbo"]);
  cmd_vel_msg.linear.z = getVal(joy_msg, axis_linear_map, scale_linear_map[which_map], "z", scale_linear_map["turbo"]);
  cmd_vel_msg.angular.z = getVal(joy_msg, axis_angular_map, scale_angular_map[which_map], "yaw", scale_angular_map["turbo"]);
  cmd_vel_msg.angular.y = getVal(joy_msg, axis_angular_map, scale_angular_map[which_map], "pitch", scale_angular_map["turbo"]);
  cmd_vel_msg.angular.x = getVal(joy_msg, axis_angular_map, scale_angular_map[which_map], "roll", scale_angular_map["turbo"]);
}

else if (track_on ==-1)
{
//getVal(joy_msg)
  double turbo_scale = (1.0 + fabs(joy_msg->axes[enable_turbo_button] * (scale_linear_map["turbo"].at("x") - scale_linear_map[which_map].at("x"))));
  float left_stick_value = joy_msg->axes[1] ; // * scale_linear_map[which_map].at("x")/2;
  float right_stick_value = joy_msg->axes[4] ; //* scale_linear_map[which_map].at("x")/2;
  float vel_x = left_stick_value + right_stick_value;
  float angular_vel_z = (1/base_width) * (right_stick_value - left_stick_value);

  
  if 
  
      ((left_stick_value < deadzone && left_stick_value > -deadzone) 
      
      && 
      
      (right_stick_value < deadzone && right_stick_value > -deadzone))
      //if BOTH joy sticks are near default(0), send no message
  {
    cmd_vel_msg.linear.x = 0;
    cmd_vel_msg.angular.z = 0;
    
  }
  else
  {
    cmd_vel_msg.linear.x = turbo_scale * vel_x * scale_linear_map[which_map].at("x")/2;
    cmd_vel_msg.angular.z = turbo_scale* angular_vel_z  *scale_linear_map[which_map].at("x")/2;
    
  }
}
  cmd_vel_pub.publish(cmd_vel_msg);
  sent_disable_msg = false;
}

void TeleopTwistJoy::Impl::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
  if (joy_msg->axes[enable_button] < -750.0)
  {

    if (joy_msg->buttons[enable_track_control_button])
    {
      if (timer > 25)
      {
        

        track_on = track_on*-1;
        timer = 0;
      }

    }
  timer ++; 
      sendCmdVelMsg(joy_msg, "normal");
  }
  else
  {
    // When enable button is released, immediately send a single no-motion command
    // in order to stop the robot.
    if (!sent_disable_msg)
    {
      // Initializes with zeros by default.
      geometry_msgs::Twist cmd_vel_msg;
      cmd_vel_pub.publish(cmd_vel_msg);
      sent_disable_msg = true;
    }
  }
}

}  // namespace teleop_twist_joy
