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

#ifndef YAMABROS_H
#define YAMABROS_H

#include <ros/ros.h>

#include <cmath>
#include <string>

namespace yamabros {

/** Conversion factor from radians to degress. */
const double RADS = 180.0 / M_PI;

/** Conversion factor from degrees to radians. */
const double DEGS = M_PI / 180.0;

template<class T> T param(const std::string &name, const T &fallback)
{
  T value;
  return (ros::param::get(name, value) ? value : fallback);
}

} // namespace yamabros

#endif
