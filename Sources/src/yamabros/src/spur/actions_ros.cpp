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

#include <yamabros/spur/actions_ros.h>

#include <yamabros/settings.h>
#include <yamabros/yamabros.h>

#include <tf/transform_broadcaster.h>

#include <boost/bind.hpp>

#include <cmath>

namespace yamabros
{

namespace spur
{

ActionsROS::ActionsROS():
  Actions(),
  node_(),
  action_client_("move_base", true),
  tt1_(0.0)
{
  cmd_vel_ = node_.advertise<geometry_msgs::Twist>(settings::spur::cmd_vel(), 1);
  odom_ = node_.subscribe<nav_msgs::Odometry>(settings::spur::odom(), 1, &ActionsROS::odom_callback, this);

  while(!action_client_.waitForServer(ros::Duration(5.0)))
    ROS_INFO_STREAM("Waiting for the move_base action server to come up...");

  ROS_INFO_STREAM("Waiting for the move_base action server to come up... Done");
}

void ActionsROS::odom_callback(const nav_msgs::Odometry::ConstPtr& odometry)
{
  Pose &pose = this->pose();
  pose.x = odometry->pose.pose.position.x;
  pose.y = odometry->pose.pose.position.y;

  /*
  The management of pose orientation is more complicated for the ROS implementation
  because yaw is reported as an angle between -pi and pi, but what we really want
  is an absolute value that increases / decreases indefinitely with each additional
  turn. In order to convert between the two formats it's necessary to keep track of
  the instantaneous angle and accumulate changes over time.
  */

  double t = tf::getYaw(odometry->pose.pose.orientation);

  // If the difference between previous and current orientations is greater than pi,
  // it means there was an overflow from pi to -pi or vice-versa.
  double dt = t - tt1_;
  double dm = fabs(dt);
  if (dm >= M_PI)
    dt = copysign(1.0, dt) * (dm - 2 * M_PI);

  pose.t += dt;
  tt1_ = t;

  ROS_INFO_STREAM("Pose : (" << pose.x << ", " << pose.y << ", " << pose.t << ")");
}

static void forward(Actions *actions,
                    const actionlib::SimpleClientGoalState& state,
                    const move_base_msgs::MoveBaseResultConstPtr& result)
{
  double v = settings::spur::lin_vel();
  if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
    actions->set(v, 0);
}

void ActionsROS::approach(double x, double y, double t)
{
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(t);

  action_client_.sendGoal(goal, boost::bind(forward, this, _1, _2));
}

void ActionsROS::brake()
{
  set(0, 0);
}

static void encircle(Actions *actions, double v, double w,
                     const actionlib::SimpleClientGoalState& state,
                     const move_base_msgs::MoveBaseResultConstPtr& result)
{
  if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
    actions->set(v, w);
}

void ActionsROS::circle(double x, double y, double r)
{
  /*
  This implementation works by computing a point tangential to the desired
  circle, moving the robot there, and then setting the linear and angular
  speeds so the robot moves along a circle of the desired radius.

  The robot will circle counter-clockwise for a positive radius, and clockwise
  for a negative one.
  */

  double sx = copysign(1.0, x);
  double sy = copysign(1.0, y);
  double sr = copysign(1.0, r);

  /*
  Center of circle    Position of tangent
  ---------------------------------------
  x > 0, y > 0        (x, y - r, 0.0)
  x > 0, y < 0        (x, y - r, 0.0)
  x < 0, y > 0        (x, y + r,  pi)
  x < 0, y < 0        (x, y + r, -pi)
  */
  double xt = x;
  double yt = y - sx * r;
  double tt = (x > 0 ? 0.0 : sy * M_PI);

  // The angular speed over a circle of radius r is the ratio
  // between the radius and the linear speed.
  double v = settings::spur::lin_vel();
  double w = v / r;

  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = xt;
  goal.target_pose.pose.position.y = yt;
  goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(tt);

  action_client_.sendGoal(goal, boost::bind(encircle, this, v, w, _1, _2));
}

void ActionsROS::coast()
{
  set(0, 0);
}

void ActionsROS::set(double v, double w)
{
  if (!action_client_.getState().isDone())
    action_client_.cancelGoal();

  geometry_msgs::Twist twist;
  twist.linear.x = v;
  twist.linear.y = 0;
  twist.linear.z = 0;
  twist.angular.x = 0;
  twist.angular.y = 0;
  twist.angular.z = w;

  cmd_vel_.publish(twist);
}

void ActionsROS::spin(double t)
{
  double w = copysign(1.0, t) * settings::spur::ang_vel();

  set(0, w);
}

void ActionsROS::straight()
{
  double v = settings::spur::lin_vel();

  set(v, 0);
}

} // namespace spur

} // namespace yamabros
