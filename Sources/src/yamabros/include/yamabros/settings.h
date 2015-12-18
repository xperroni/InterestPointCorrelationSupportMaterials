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

#ifndef YAMABROS_SETTINGS_H
#define YAMABROS_SETTINGS_H

#include <string>

namespace yamabros
{

namespace settings
{

double hz();

namespace spur
{

std::string cmd_vel();

std::string odom();

std::string device();

std::string param_file();

double ang_acc();

double ang_vel();

double lin_acc();

double lin_vel();

int retries();

double retry_delay();

std::string server_type();

} // namespace spur

} // namespace settings

} // namespace yamabros

#endif
