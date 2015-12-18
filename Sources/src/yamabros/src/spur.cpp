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

#include <yamabros/spur.h>

#include <geometry_msgs/Twist.h>

namespace yamabros
{

namespace spur
{

Spur::Spur(bool blocking):
  node_(),
  action_client_(new ActionClient(node_.resolveName("spur"), true)),
  blocking_(blocking)
{
  action_client_->waitForServer();

  std::string root = node_.resolveName("spur");
  std::string cmd_vel_topic = ros::names::append(root, node_.resolveName("cmd_vel"));
  cmd_vel_ = node_.advertise<geometry_msgs::Twist>(cmd_vel_topic, 100, true);
}

void Spur::send(Command command)
{
  CommandGoal goal;
  goal.command = command;
  action_client_->sendGoal(goal);
  action_client_->waitForResult();
}

void Spur::send(Command command, Mode mode, int size, ...)
{
  va_list args;
  va_start(args, size);

  CommandGoal goal;
  goal.command = command;
  std::vector<double> &destination = goal.destination;
  for (int i = 0; i < size; i++)
    destination.push_back(va_arg(args, double));

  action_client_->sendGoal(goal);
  if (mode == BLOCK || (mode == DEFAULT && blocking_))
    action_client_->waitForResult();

  va_end(args);
}

void Spur::brake()
{
  send(BRAKE);
}

void Spur::coast()
{
  send(COAST);
}

void Spur::steer(double v, double w)
{
  geometry_msgs::Twist twist;
  twist.linear.x = v;
  twist.angular.z = w;
  cmd_vel_.publish(twist);
}

void Spur::straight()
{
  approach(0.1, 0, ASYNC);
}

void Spur::straight(double d, Mode mode)
{
  approach(d, 0, mode);
}

void Spur::approach(double x, double y, Mode mode)
{
  send(APPROACH, mode, 2, x, y);
}

void Spur::approach(double x, double y, double t, Mode mode)
{
  send(APPROACH, mode, 3, x, y, t);
}

void Spur::circle(double x, double y, double r, Mode mode)
{
  send(CIRCLE, mode, 3, x, y, r);
}

void Spur::circle(double x, double y, double r, double t, Mode mode)
{
  send(CIRCLE, mode, 4, x, y, r, t);
}

void Spur::circle(double x, double y, double r, double t, double e, Mode mode)
{
  send(CIRCLE, mode, 5, x, y, r, t, e);
}

void Spur::spin(double t, Mode mode)
{
  send(SPIN, mode, 1, t);
}

void Spur::spin(double t, double e, Mode mode)
{
  send(SPIN, mode, 2, t, e);
}

void Spur::turn(double t, Mode mode)
{
  send(TURN, mode, 1, t);
}

void Spur::turn(double t, double e, Mode mode)
{
  send(TURN, mode, 2, t, e);
}

bool Spur::active()
{
  return (action_client_->getState() == actionlib::SimpleClientGoalState::ACTIVE);
}

Pose Spur::pose()
{
  send(POSE);
  return action_client_->getResult()->pose;
}

} // namespace spur

} // namespace yamabros
