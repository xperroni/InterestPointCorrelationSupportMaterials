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

#ifndef DEJAVU_DISPATCHER_H
#define DEJAVU_DISPATCHER_H

#include <boost/function.hpp>

#include <ros/ros.h>

#include <map>

namespace dejavu
{

class Dispatcher
{
  /** \brief Reference to the current ROS node. */
  ros::NodeHandle node_;

  /** \brief Publishers instantiated by this node. */
  std::map<std::string, ros::Publisher> publishers_;

  /** \brief Subscribers instantiated by this node. */
  std::map<std::string, ros::Subscriber> subscribers_;

public:
  std::string name(const std::string &placeholder);

  template<class T>
  void publish(const std::string &topic, const T &value, bool latch = false);

  template<class T, class F>
  void subscribe(const std::string &topic, F callback);

  template<class T, class C>
  void subscribe(const std::string &topic, void(C::*callback)(const boost::shared_ptr<T const>&), C *object);

  template<class T>
  T get(const std::string &name, const T &fallback);

  template<class T>
  void set(const std::string &name, const T &value);
};

template<class T>
void Dispatcher::publish(const std::string &topic, const T &value, bool latch)
{
  if (publishers_.count(topic) == 0 || publishers_[topic].isLatched() != latch)
    publishers_[topic] = node_.advertise<T>(topic, 100, latch);

  publishers_[topic].publish(value);
}

template<>
void Dispatcher::publish(const std::string &topic, const bool &value, bool latch);

template<>
void Dispatcher::publish(const std::string &topic, const int &value, bool latch);

template<>
void Dispatcher::publish(const std::string &topic, const std::string &value, bool latch);

template<class T, class F>
void Dispatcher::subscribe(const std::string &topic, F callback)
{
  subscribers_[topic] = node_.subscribe<T>(topic, 1, callback);
}

template<class T, class C>
void Dispatcher::subscribe(const std::string &topic, void(C::*callback)(const boost::shared_ptr<T const>&), C *object)
{
  subscribers_[topic] = node_.subscribe<T>(topic, 1, callback, object);
}

template<class T>
T Dispatcher::get(const std::string &name, const T &fallback)
{
  T value;
  return (ros::param::get(name, value) ? value : fallback);
}

template<class T>
void Dispatcher::set(const std::string &name, const T &value)
{
  ros::param::set(name, value);
}

} // namespace dejavu

#endif
