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

#include <yamabros/spur/topics.h>

#include <yamabros/yamabros.h>

#include <nav_msgs/Odometry.h>

#include <iostream>

namespace yamabros
{

namespace spur
{

Topics::~Topics()
{
  // Nothing to do.
}

void Topics::spinOnce()
{
  // Empty function, implement on subclasses.
}

} // namespace spur

} // namespace yamabros
