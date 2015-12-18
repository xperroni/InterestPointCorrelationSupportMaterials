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

#ifndef YAMABROS_SPUR_SERVER_H
#define YAMABROS_SPUR_SERVER_H

#include <yamabros/spur/actions.h>
#include <yamabros/spur/topics.h>

namespace yamabros
{

namespace spur
{

class Server
{
  /** \brief Spur action server. */
  Actions::Ptr actions_;

  /** \brief Spur topic server. */
  Topics::Ptr topics_;

public:
  Server();

  void spin();
};

} // namespace spur

} // namespace yamabros

#endif
