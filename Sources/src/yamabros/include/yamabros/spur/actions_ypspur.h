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

#ifndef YAMABROS_SPUR_ACTIONS_YPSPUR_H
#define YAMABROS_SPUR_ACTIONS_YPSPUR_H

#include <yamabros/spur/actions.h>

namespace yamabros
{

namespace spur
{

class ActionsYPSPUR: public Actions
{
public:
  // See Actions::approach
  virtual void approach(double x, double y, double t);

  // See Actions::brake
  virtual void brake();

  // See Actions::circle
  virtual void circle(double x, double y, double r);

  // See Actions::coast
  virtual void coast();

  // See Actions::set
  virtual void set(double v, double w);

  // See Actions::spin
  virtual void spin(double t);

  // See Actions::straight
  virtual void straight();

  // See Actions::pose
  virtual Pose &pose();
};

} // namespace spur

} // namespace yamabros

#endif
