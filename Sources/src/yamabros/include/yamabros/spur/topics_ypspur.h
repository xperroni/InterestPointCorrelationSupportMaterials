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

#ifndef YAMABROS_SPUR_TOPICS_YPSPUR_H
#define YAMABROS_SPUR_TOPICS_YPSPUR_H

#include <yamabros/spur/coordinator.h>
#include <yamabros/spur/topics.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

namespace yamabros
{

namespace spur
{

class TopicsYPSPUR: public Topics
{
  /** \brief ROS node handler. */
  ros::NodeHandle node_;

  /** \brief ypspur-coordinator background process handler. */
  Coordinator coordinator_;

  /** \brief Odometry data broadcaster. */
  tf::TransformBroadcaster odom_broadcaster_;

  /** \brief Odometry data publisher. */
  ros::Publisher odom_publisher_;

  /** \brief Input topic for linear and angular speeds. */
  ros::Subscriber cmd_vel_;

  /** \brief How many more log messages will be skipped before the next output */
  int log_skips_;

  void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& twist);

  void publish_odometry();

public:
  TopicsYPSPUR();

  virtual ~TopicsYPSPUR();

  /**
   * \brief Run pending callbacks.
   */
  virtual void spinOnce();
};

} // namespace spur

} // namespace yamabros

#endif
