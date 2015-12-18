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

#include <yamabros/spur/server.h>

#include <yamabros/yamabros.h>
#include <yamabros/settings.h>
#include <yamabros/spur/actions_ros.h>
#include <yamabros/spur/actions_ypspur.h>
#include <yamabros/spur/topics_ypspur.h>

#include <iostream>
#include <cmath>

namespace yamabros
{

namespace spur
{

Server::Server()
{
  std::string server_type = settings::spur::server_type();
  if (server_type == "ypspur")
  {
    topics_.reset(new TopicsYPSPUR());
    actions_.reset(new ActionsYPSPUR());
  }
  else if (server_type == "ypspur_ros")
  {
    topics_.reset(new TopicsYPSPUR());
    actions_.reset(new ActionsROS());
  }
  else if (server_type == "ros")
  {
    topics_.reset(new Topics()); // Dummy implementation.
    actions_.reset(new ActionsROS());
  }
  else
  {
    std::string msg = "Unknown server type \"" + server_type + "\"";
    ROS_ERROR_STREAM(msg);
    throw std::runtime_error(msg);
  }
}

void Server::spin()
{
  // Update rate, default 100 Hz
  double hz = param<double>("~hz", 100);

  for (ros::Rate rate(hz); ros::ok();)
  {
    ros::spinOnce();
    actions_->spinOnce();
    topics_->spinOnce();
    rate.sleep();
  }
}

} // namespace spur

} // namespace yamabros
