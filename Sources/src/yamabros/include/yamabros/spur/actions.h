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

#ifndef YAMABROS_SPUR_ACTIONS_H
#define YAMABROS_SPUR_ACTIONS_H

#include <yamabros/CommandAction.h>
#include <yamabros/Pose.h>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <boost/function.hpp>

#include <vector>

namespace yamabros
{

namespace spur
{

/** \brief Action completion condition type. */
typedef boost::function<bool()> Condition;

class Actions
{
  /** \brief Action handler type. */
  typedef boost::function<void(const std::vector<double>&)> Action;

  /** \brief Command implementor type. */
  struct Command
  {
    /** \brief Smart pointer alias type. */
    typedef boost::shared_ptr<Command> Ptr;

    /** \brief Command's name. */
    std::string name;

    /** \brief Minimum argument count. */
    int least;

    /** \brief Command's action. */
    Action run;

    /**
     * \brief Default constructor.
     */
    Command()
    {
      // Nothing to do.
    }

    /**
     * \brief Wraps a custom function into a State object.
     */
    Command(const std::string &name_, int least_, Action run_):
      name(name_),
      least(least_),
      run(run_)
    {
      // Nothing to do.
    }
  };

  /** \brief Vector of spur commands. */
  std::vector<Command::Ptr> commands_;

  /** \brief Currently running command. */
  Command::Ptr command_;

  /** \brief ROS node handler. */
  ros::NodeHandle node_;

  /** \brief Spur command action server. */
  actionlib::SimpleActionServer<CommandAction> action_server_;

  /** \brief Ongoing command's feedback handler. */
  boost::function<void()> command_feedback_;

  /** \brief Current command feedback. */
  CommandFeedback feedback_;

  /** \brief Last command result. */
  CommandResult result_;

  /** \brief Current robot pose. */
  Pose pose_;

  /*
   * ****************************************************************************
   * Action management
   * ****************************************************************************
   */

  /**
   * \brief Accept an action from a client.
   */
  void accept();

  /**
   * \brief Reset the server's state.
   */
  void clear();

  /**
   * \brief Configure a command.
   */
  void configure(const std::string &name, int least, void(Actions::*action)(const std::vector<double>&));

  /**
   * \brief Preempt an ongoing action.
   */
  void preempt();

  /**
   * \brief Update the current feedback.
   *
   * \return Reference to the Feedback object.
   */
  CommandFeedback &updateFeedback();

  /**
   * \brief Update the current Result.
   *
   * \return Reference to the Result object.
   */
  CommandResult &updateResult();

  /*
   * ****************************************************************************
   * Action handlers
   * ****************************************************************************
   */

  /**
   * \brief Send the robot towards a given pose.
   */
  void approach(const std::vector<double> &destination);

  /**
   * \brief Stop the robot dead on its wheels.
   */
  void brake(const std::vector<double> &destination);

  /**
   * \brief Set the robot in a circular path around the given point.
   */
  void circle(const std::vector<double> &destination);

  /**
   * \brief Stop the robot smoothly.
   */
  void coast(const std::vector<double> &destination);

  /**
   * \brief Report the current pose.
   */
  void pose(const std::vector<double> &destination);

  /**
   * \brief Spin the robot by a given angle.
   */
  void spin(const std::vector<double> &destination);

  /**
   * \brief Turn the robot toward a given side while also advancing.
   */
  void turn(const std::vector<double> &destination);

protected:
  /**
   * \brief Provide feedback for an ongoing action.
   *
   * If the given \c finished callable returns true, set the action as succeeded.
   */
  void feedback(Condition finished);

  /**
   * \brief Return the value at the given index if possible, or the fall-back value otherwise.
   */
  double get(const std::vector<double> &destination, int index, double fallback);

  /**
   * \brief Set the termination condition for an initiated action.
   */
  void wait(Condition finished);

public:
  typedef boost::shared_ptr<Actions> Ptr;

  Actions();

  /**
   * \brief Run pending callbacks.
   */
  virtual void spinOnce();

  /**
   * \brief Send the robot towards the given pose.
   *
   * Coordinates are given relative to the present pose.
   */
  virtual void approach(double x, double y, double t) = 0;

  /**
   * \brief Stop the robot dead on its wheels.
   *
   * This command should lock the wheels, forcing the robot to stop immediately.
   */
  virtual void brake() = 0;

  /**
   * \brief Set the robot in a circular path around the given point.
   *
   * The robot circles the center point at distance <i>r</i>m.
   * Center coordinates are relative to the current pose.
   */
  virtual void circle(double x, double y, double r) = 0;

  /**
   * \brief Stop the robot smoothly.
   *
   * This command should cut power to the wheels but not lock them, allowing the
   * robot to smoothly come to a stop as friction decreases its speed.
   */
  virtual void coast() = 0;

  /**
   * \brief Set the robot's linear and angular speeds.
   */
  virtual void set(double v, double w) = 0;

  /**
   * \brief Spin the robot by the given angle.
   *
   * Positive values spin the robot counter-clockwise ("to the left"), and negative
   * ones, clockwise ("to the right").
   */
  virtual void spin(double t) = 0;

  /**
   * \brief Set the robot in a straight trajectory.
   */
  virtual void straight() = 0;

  /**
   * \brief Return the current pose.
   *
   * \return Reference to the Pose object.
   */
  virtual Pose &pose();
};

} // namespace spur

} // namespace yamabros

#endif
