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

#include <yamabros/yamabros.h>
#include <yamabros/settings.h>

namespace yamabros
{

namespace settings
{

double hz()
{
  return param<double>("~hz", 100);
}

namespace spur
{

static std::string topic(const std::string &name)
{
  ros::NodeHandle node;
  return node.resolveName(name);
}

std::string cmd_vel()
{
  return topic("cmd_vel");
}

std::string odom()
{
  return topic("odom");
}

std::string device()
{
  return param<std::string>("~device", "/dev/ttyACM1");
}

std::string param_file()
{
  return param<std::string>("~param_file", "/usr/local/share/robot-params/M1.param");
}

double ang_acc()
{
  return param<double>("~ang_acc", 2.0);
}

double ang_vel()
{
  return param<double>("~ang_vel", 1.5);
}

double lin_acc()
{
  return param<double>("~lin_acc", 1.0);
}

double lin_vel()
{
  return param<double>("~lin_vel", 0.3);
}

int retries()
{
  return param<int>("~retries", 100);
}

double retry_delay()
{
  return param<double>("~retry_delay", 1.0);
}

std::string server_type()
{
  return param<std::string>("~server_type", "ypspur");
}

} // namespace spur

} // namespace settings

} // namespace yamabros
