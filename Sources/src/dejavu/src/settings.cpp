/*
 * Copyright (c) Helio Perroni Filho <xperroni@gmail.com>
 *
 * This file is part of Dejavu.
 *
 * Dejavu is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Dejavu is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Dejavu. If not, see <http://www.gnu.org/licenses/>.
 */

#include <dejavu/settings.h>

#include <boost/filesystem.hpp>
using boost::filesystem::current_path;

namespace dejavu
{

namespace name
{

std::string name(const std::string &name)
{
  return ros::names::resolve(name);
}

std::string direction()
{
  return name("direction");
}

std::string forward()
{
  return name("forward");
}

std::string image()
{
  return name("image");
}

std::string image_difference()
{
  return ros::names::append(image(), name("difference"));
}

std::string image_matching()
{
  return ros::names::append(image(), matching());
}

std::string matching()
{
  return name("matching");
}

} // namespace name

namespace param
{

double difference_threshold()
{
  return param<double>("~difference_threshold", 15.0);
}

int divs_bins()
{
  return param<int>("~divs_bins", 50);
}

int divs_width()
{
  return param<int>("~divs_width", 16);
}

std::string path()
{
  return param<std::string>("~path", "");
}

std::string path_ground_truth()
{
  return param<std::string>("~path_ground_truth", (current_path() / "ground_truth.txt").native());
}

std::string path_matches()
{
  return param<std::string>("~path_matches", (current_path() / "matches.txt").native());
}

std::string path_teach()
{
  return param<std::string>("~path_teach", "");
}

std::string path_replay()
{
  return param<std::string>("~path_replay", "");
}

int teach_images()
{
  return param<int>("~teach_images", 25);
}

int teach_padding()
{
  return param<int>("~teach_padding", 30);
}

int replay_images()
{
  return param<int>("~replay_images", 25);
}

double replay_leak()
{
  return param<double>("~replay_leak", 0.98);
}

int replay_padding()
{
  return param<int>("~replay_padding", 30);
}

} // namespace param

} // namespace dejavu

