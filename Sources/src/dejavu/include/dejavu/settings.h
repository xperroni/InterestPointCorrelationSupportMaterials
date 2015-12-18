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

#ifndef DEJAVU_SETTINGS_H
#define DEJAVU_SETTINGS_H

#include <ros/ros.h>

#include <string>

namespace dejavu
{

namespace name
{

std::string name(const std::string &name);

std::string direction();

std::string forward();

std::string image();

std::string image_difference();

std::string image_matching();

std::string matching();

} // namespace name

namespace param
{

template<class T> T param(const std::string& name, const T& fallback)
{
  T value;
  return (ros::param::get(name, value) ? value : fallback);
}

double difference_threshold();

int divs_bins();

int divs_width();

std::string path();

std::string path_ground_truth();

std::string path_matches();

std::string path_teach();

std::string path_replay();

int teach_images();

int teach_padding();

int replay_images();

double replay_leak();

int replay_padding();

} // namespace param

} // namespace dejavu

#endif