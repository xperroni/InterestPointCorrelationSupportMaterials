/*
This file is part of Yamabros.

Yamabros is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

Yamabros is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Yamabros. If not, see <http://www.gnu.org/licenses/>.
*/

#include <yamabros/spur/topics_ypspur.h>

#include <yamabros/yamabros.h>
#include <yamabros/settings.h>

#include <ypspur.h>

#include <nav_msgs/Odometry.h>

#include <iostream>
#include <cmath>

namespace yamabros
{

namespace spur
{

TopicsYPSPUR::TopicsYPSPUR():
  node_(),
  coordinator_(),
  odom_broadcaster_(),
  log_skips_(0)
{
  int status = Spur_init();
  if (status != 1)
  {
    std::string msg = "Spur_init() failed";
    ROS_ERROR_STREAM(msg);
    throw std::runtime_error(msg);
  }

  Spur_set_accel(settings::spur::lin_acc());
  Spur_set_vel(settings::spur::lin_vel());

  Spur_set_angaccel(settings::spur::ang_acc());
  Spur_set_angvel(settings::spur::ang_vel());

  Spur_set_pos_GL(0, 0, 0);

  odom_publisher_ = node_.advertise<nav_msgs::Odometry>(settings::spur::odom(), 100);

  cmd_vel_ = node_.subscribe<geometry_msgs::Twist>(settings::spur::cmd_vel(), 1, &TopicsYPSPUR::cmd_vel_callback, this);
}

TopicsYPSPUR::~TopicsYPSPUR()
{
  Spur_free();
}

void TopicsYPSPUR::cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& twist)
{
  double v = twist->linear.x;
  double w = twist->angular.z;
  Spur_vel(v, w);

  ROS_INFO_STREAM(std::fixed << "cmd: " << v << " [m/s], " << ", " << (w * RADS) << " [deg/s]");
}

void TopicsYPSPUR::publish_odometry()
{
  ros::Time now = ros::Time::now();

  double x = 0.0, y = 0.0, t = 0.0;
  Spur_get_pos_GL(&x, &y, &t);

  double v = 0.0, w = 0.0;
  Spur_get_vel(&v, &w);

  // Robot orientation in quaternion notation
  // See: http://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
  geometry_msgs::Quaternion orientation = tf::createQuaternionMsgFromYaw(t);

  // tf
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = now;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";
  odom_trans.transform.translation.x = x;
  odom_trans.transform.translation.y = y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = orientation;
  odom_broadcaster_.sendTransform(odom_trans);

  // nav_msgs
  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = now;
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base_link";
  odom_msg.pose.pose.position.x = x;
  odom_msg.pose.pose.position.y = y;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation = orientation;
  odom_msg.twist.twist.linear.x = v;
  odom_msg.twist.twist.linear.y = 0.0;
  odom_msg.twist.twist.linear.z = 0.0;
  odom_msg.twist.twist.angular.x = 0.0;
  odom_msg.twist.twist.angular.y = 0.0;
  odom_msg.twist.twist.angular.z = w;
  odom_publisher_.publish(odom_msg);

  // Log odometry data
  if (--log_skips_ <= 0)
  {
    log_skips_ = param<int>("~log_skips", 100);
    ROS_INFO_STREAM(std::fixed
                    << "vel: " << v << " [m/s], " << (w * RADS) << " [deg/s], odom: "
                    << x << " [m], " << y << " [m], " << (t * RADS) << " [deg]");
  }
}

void TopicsYPSPUR::spinOnce()
{
  publish_odometry();
}

} // namespace spur

} // namespace yamabros
