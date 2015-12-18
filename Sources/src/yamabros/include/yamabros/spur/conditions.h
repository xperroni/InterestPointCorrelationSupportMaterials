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

#ifndef YAMABROS_CONDITIONS_H
#define YAMABROS_CONDITIONS_H

#include <yamabros/spur/actions.h>

#include <boost/function.hpp>

namespace yamabros
{

namespace spur
{

/**
 * \brief Facility function for actions that never succeed.
 */
bool forever();

/**
 * \brief Facility function for actions that succeed immediately.
 */
bool completed();

/**
 * \brief Condition to check for when the robot has moved past a given distance.
 *
 * Given a distance relative to the current pose, this function computes a global
 * location and creates a Condition object to watch for it being passed.
 *
 * \arg actions Pointer to action server.
 *
 * \arg d2 Target distance squared.
 *
 * \return Condition to check for.
 */
Condition farther(Actions *actions, double d2);

/**
 * \brief Condition to check for when the robot turned past a given angle.
 *
 * Given an orientation angle relative to the current pose, this function computes a
 * global orientation and creates a Condition object to watch for it being reached.
 *
 * \arg actions Pointer to action server.
 *
 * \arg angle Target robot orientation relative to current pose, in radians.
 *
 * \arg error Error margin, in radians.
 *
 * \return Condition to check for.
 */
Condition near(Actions *actions, double angle, double error);

/**
 * \brief Condition to limit how much the robot is going to turn.
 *
 * Given an orientation angle relative to the current pose, this function computes a
 * global orientation and creates a Condition object to watch for it being reached.
 * When that happens, a command is issued to make the robot move straight.
 *
 * \arg actions Pointer to action server.
 *
 * \arg angle Target robot orientation relative to current pose, in radians.
 *
 * \arg error Error margin, in radians.
 *
 * \return Condition to check for.
 */
Condition turned(Actions *actions, double angle, double error);

} // namespace spur

} // namespace yamabros

#endif
