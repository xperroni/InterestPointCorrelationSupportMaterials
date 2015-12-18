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

#ifndef YAMABROS_SPUR_COORDINATOR_H
#define YAMABROS_SPUR_COORDINATOR_H

#include <cstdio>
#include <string>

namespace yamabros
{

namespace spur
{

class Coordinator
{
  /** \brief Communication channel to the ypspur-coordinator executable. */
  FILE *pipe_;

  /**
   * \brief Read a line from the communication channel.
   *
   * This operation is non-blocking: it always returns immediately, regardless of whether
   * a full line was read or not.
   *
   * \arg line String variable to hold the read line.
   *
   * \return \c true on success, \c false otherwise.
   */
  bool read(std::string &line);

public:
  Coordinator();

  virtual ~Coordinator();
};

} // namespace spur

} // namespace yamabros

#endif
