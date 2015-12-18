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

#include <yamabros/spur/conditions.h>

#include <boost/bind.hpp>

namespace yamabros
{

namespace spur
{

bool forever()
{
  return false;
}

bool completed()
{
  return true;
}

static bool farther_(Actions *actions, double x0, double y0, double d2)
{
  Pose &pose = actions->pose();
  double x = pose.x;
  double y = pose.y;

  return (pow(x - x0, 2.0) + pow(y - y0, 2.0) >= d2);
}

Condition farther(Actions *actions, double d2)
{
  Pose &pose = actions->pose();
  return boost::bind(farther_, actions, pose.x, pose.y, d2);
}

/**
 * \brief Check whether the robot's current orientation is near a given angle.
 *
 * \arg pose Current robot pose.
 *
 * \arg orientation Target robot orientation, in radians.
 *
 * \arg direction either \c 1 or \c -1; indicates whether the robot is turning
 *                counter-clockwise (leftwards) or clockwise (rightwards), respectively.
 *
 * \arg error Error margin, in radians.
 *
 * \return \c true if the current robot orientation is within \c error radians
 * of \c angle; \c false otherwise.
 */
static bool near_(Actions *actions, double orientation, double direction, double error)
{
  double t = actions->pose().t;
  double tt = (t - orientation) * direction;
  return (tt > 0 || -tt <= error);
}

Condition near(Actions *actions, double angle, double error)
{
  double direction = copysign(1.0, angle);
  double orientation = actions->pose().t + angle;
  return boost::bind(near_, actions, orientation, direction, error);
}

static bool turned_(Actions *actions, double orientation, double direction, double error)
{
  bool neared = near_(actions, orientation, direction, error);
  if (neared)
    actions->straight();

  return neared;
}

Condition turned(Actions *actions, double angle, double error)
{
  double direction = copysign(1.0, angle);
  double orientation = actions->pose().t + angle;
  return boost::bind(turned_, actions, orientation, direction, error);
}

} // namespace spur

} // namespace yamabros
