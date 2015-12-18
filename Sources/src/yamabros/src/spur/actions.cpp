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

#include <yamabros/spur/actions.h>

#include <yamabros/yamabros.h>
#include <yamabros/settings.h>
#include <yamabros/spur/conditions.h>
#include <yamabros/spur/constants.h>

#include <boost/bind.hpp>

#include <cmath>

namespace yamabros
{

namespace spur
{

Actions::Actions():
  node_(),
  action_server_(node_, node_.resolveName("spur"), false)
{
  configure("approach", 2, &Actions::approach);
  configure("brake", 0, &Actions::brake);
  configure("circle", 3, &Actions::circle);
  configure("coast", 0, &Actions::coast);
  configure("pose", 0, &Actions::pose);
  configure("spin", 1, &Actions::spin);
  configure("turn", 1, &Actions::turn);

  action_server_.registerGoalCallback(boost::bind(&Actions::accept, this));
  action_server_.registerPreemptCallback(boost::bind(&Actions::preempt, this));

  clear();

  action_server_.start();
}

void Actions::accept()
{
  CommandGoalConstPtr goal = action_server_.acceptNewGoal();
  if (action_server_.isPreemptRequested())
    return;

  command_ = commands_[goal->command];
  const std::string &name = command_->name;
  int least = command_->least;

  const std::vector<double> &destination = goal->destination;
  int size = destination.size();

  if (size >= least)
  {
    ROS_INFO_STREAM(name << "() started");
    command_->run(goal->destination);
  }
  else
  {
    std::stringstream message;
    message << "Error calling " << name << "(): received " << size << " arguments, expected " << least;
    action_server_.setAborted(updateResult(), message.str());
    clear();
  }
}

static void nothing()
{
  // Nothing to do.
}

void Actions::clear()
{
  command_feedback_ = nothing;
  command_.reset();
}

void Actions::configure(const std::string &name, int least, void(Actions::*action)(const std::vector<double>&))
{
  Command::Ptr command(new Command(name, least, boost::bind(action, this, _1)));
  commands_.push_back(command);
}

void Actions::feedback(Condition finished)
{
  if (!action_server_.isActive())
    return;

  if (finished())
  {
    ROS_INFO_STREAM(command_->name << "() succeeded");
    action_server_.setSucceeded(updateResult());
    clear();
    return;
  }

  action_server_.publishFeedback(updateFeedback());
}

double Actions::get(const std::vector<double> &destination, int index, double fallback)
{
  return (destination.size() >= index ? destination[index] : fallback);
}

void Actions::preempt()
{
  ROS_INFO_STREAM(command_->name << "() preempted");
  action_server_.setPreempted();
  clear();
}

void Actions::wait(Condition finished)
{
  command_feedback_ = boost::bind(&Actions::feedback, this, finished);
}

void Actions::spinOnce()
{
  command_feedback_();
}

CommandFeedback &Actions::updateFeedback()
{
  feedback_.pose = pose();
  return feedback_;
}

CommandResult &Actions::updateResult()
{
  result_.pose = pose();
  return result_;
}

Pose &Actions::pose()
{
  return pose_;
}

/*
 * ****************************************************************************
 * Action handlers
 * ****************************************************************************
 */

void Actions::approach(const std::vector<double> &destination)
{
  double x = destination[0];
  double y = destination[1];
  double t = get(destination, 2, atan2(y, x));

  approach(x, y, t);

  double d2 = pow(x, 2.0) + pow(y, 2.0);

  wait(farther(this, d2));
}

void Actions::brake(const std::vector<double> &destination)
{
  brake();

  feedback(completed);
}

void Actions::circle(const std::vector<double> &destination)
{
  double x = destination[0];
  double y = destination[1];
  double r = destination[2];

  circle(x, y, r);

  if (destination.size() == 3)
    wait(forever);
  else
  {
    double t = destination[3];
    double e = get(destination, 4, 0.001);
    wait(near(this, t, e));
  }
}

void Actions::coast(const std::vector<double> &destination)
{
  coast();

  feedback(completed);
}

void Actions::pose(const std::vector<double> &destination)
{
  feedback(completed);
}

void Actions::spin(const std::vector<double> &destination)
{
  double t = destination[0];
  if (t == LEFT || t == RIGHT)
  {
    double s = copysign(1.0, t);
    spin(s);

    wait(forever);
  }
  else
  {
    double e = get(destination, 1, 0.001);
    spin(t);

    wait(near(this, t, e));
  }
}

void Actions::turn(const std::vector<double> &destination)
{
  double v = settings::spur::lin_vel();

  double p = 5.0 * v;
  double t = destination[0];
  double x = get(destination, 1, v);
  double y = copysign(get(destination, 2, p), t);
  double r = copysign(get(destination, 3, p), t);

  Condition finished = forever;
  if (t != LEFT && t != RIGHT)
  {
    double e = get(destination, 4, 0.001);
    finished = turned(this, t, e);
  }

  circle(x, y, r);
  wait(finished);
}

} // namespace spur

} // namespace yamabros