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

#ifndef YAMABROS_SPUR_SPUR_H
#define YAMABROS_SPUR_SPUR_H

#include <yamabros/CommandAction.h>
#include <yamabros/Pose.h>
#include <yamabros/yamabros.h>
#include <yamabros/spur/constants.h>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

namespace yamabros
{

namespace spur
{

class Spur
{
  typedef actionlib::SimpleActionClient<CommandAction> ActionClient;

  /** \brief Action invocation modes. */
  enum Mode
  {
    BLOCK,
    ASYNC,
    DEFAULT
  };

  /** \brief List of command ID's. */
  enum Command
  {
    APPROACH,
    BRAKE,
    CIRCLE,
    COAST,
    POSE,
    SPIN,
    TURN
  };

  /** \brief ROS node handler. */
  ros::NodeHandle node_;

  /** \brief Spur command action client. */
  boost::shared_ptr<ActionClient> action_client_;

  /** \brief Access to the robot's speed settings. */
  ros::Publisher cmd_vel_;

  /** \brief Whether action requests should block by default. */
  bool blocking_;

  /**
   * \brief Send a synchronous, no-arguments goal to the server.
   */
  void send(Command command);

  /**
   * \brief Send a command with given number of arguments to the server.
   *
   * If mode is BLOCK -- or alternatively, if mode is DEFAULT and this object blocks
   * by default -- this funtion blocks until the command succeeds.
   */
  void send(Command command, Mode mode, int size, ...);

public:
  /**
   * \brief Default constructor.
   */
  Spur(bool blocking = true);

  void brake();

  void coast();

  void steer(double v, double w);

  void straight();

  void straight(double d, Mode mode = DEFAULT);

  void approach(double x, double y, Mode mode = DEFAULT);

  void approach(double x, double y, double t, Mode mode = DEFAULT);

  void circle(double x, double y, double r, Mode mode = DEFAULT);

  void circle(double x, double y, double r, double t, Mode mode = DEFAULT);

  void circle(double x, double y, double r, double t, double e, Mode mode = DEFAULT);

  void spin(double t, Mode mode = DEFAULT);

  void spin(double t, double e, Mode mode = DEFAULT);

  void turn(double t, Mode mode = DEFAULT);

  void turn(double t, double e, Mode mode = DEFAULT);

  /**
   * \brief Returns whether there is a command currently in execution.
   */
  bool active();

  Pose pose();
};

} // namespace spur

} // namespace yamabros

#endif
