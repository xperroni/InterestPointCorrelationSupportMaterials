/*
Copyright (c) Helio Perroni Filho <xperroni@gmail.com>

This file is part of Dejavu.

Dejavu is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

Dejavu is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Dejavu. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef DEJAVU_MEMORY_H
#define DEJAVU_MEMORY_H

#include <clarus/core/list.hpp>

namespace dejavu
{

template<class T>
class CircularBuffer
{
  List<T> buffer_;

  size_t size_;

public:
  WorkingMemory(size_t size);

  T &append(const T &image);

  bool idle();

  int limit();
};

template<class T>
WorkingMemory<T>::WorkingMemory(int limit):
  clarus::List<T>(),
  limit_(limit)
{
  // Nothing to do.
}

template<class T>
T &WorkingMemory<T>::append(const T &image)
{
  T &last = clarus::List<T>::append(image);
  if (clarus::List<T>::size() > limit_)
    clarus::List<T>::remove(0);

  return last;
}

template<class T>
bool WorkingMemory<T>::idle()
{
  return (clarus::List<T>::size() < limit_);
}

template<class T>
int WorkingMemory<T>::limit()
{
  return limit_;
}

} // namespace dejavu

#endif
