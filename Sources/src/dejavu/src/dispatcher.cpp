/*
Copyright (c) Helio Perroni Filho <xperroni@gmail.com>

This file is part of Dejavu.

Dejavu is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

Dejavu is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Dejavu. If not, see <http://www.gnu.org/licenses/>.
*/

#include <dejavu/dispatcher.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

namespace dejavu
{

std::string Dispatcher::name(const std::string &placeholder)
{
  return node_.resolveName(placeholder);
}

template<>
void Dispatcher::publish(const std::string &topic, const bool &value, bool latch)
{
  std_msgs::Bool message;
  message.data = value;

  publish<std_msgs::Bool>(topic, message, latch);
}

template<>
void Dispatcher::publish(const std::string &topic, const int &value, bool latch)
{
  std_msgs::Int32 message;
  message.data = value;

  publish<std_msgs::Int32>(topic, message, latch);
}

template<>
void Dispatcher::publish(const std::string &topic, const std::string &value, bool latch)
{
  std_msgs::String message;
  message.data = value;

  publish<std_msgs::String>(topic, message, latch);
}

}
