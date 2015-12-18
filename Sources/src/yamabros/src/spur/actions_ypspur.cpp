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

#include <yamabros/spur/actions_ypspur.h>

#include <yamabros/yamabros.h>
#include <yamabros/spur/constants.h>

#include <ypspur.h>

#include <boost/bind.hpp>

#include <cmath>
#include <sstream>

namespace yamabros
{

namespace spur
{

void ActionsYPSPUR::approach(double x, double y, double t)
{
  YPSpur_line(CS_FS, x, y, t);
}

void ActionsYPSPUR::brake()
{
  Spur_stop();
}

void ActionsYPSPUR::circle(double x, double y, double r)
{
  YPSpur_circle(CS_FS, x, y, r);
}

void ActionsYPSPUR::coast()
{
  Spur_free();
}

void ActionsYPSPUR::set(double v, double w)
{
  Spur_vel(v, w);
}

void ActionsYPSPUR::spin(double t)
{
  YPSpur_spin(CS_FS, t);
}

void ActionsYPSPUR::straight()
{
  double v, w;
  Spur_get_vel(&v, &w);
  Spur_vel(v, 0);
}

Pose &ActionsYPSPUR::pose()
{
  Pose &pose = Actions::pose();
  Spur_get_pos_GL(&pose.x, &pose.y, &pose.t);
  return pose;
}

} // namespace spur

} // namespace yamabros