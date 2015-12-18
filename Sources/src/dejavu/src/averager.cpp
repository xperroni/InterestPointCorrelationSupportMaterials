/*
Copyright (c) Helio Perroni Filho <xperroni@gmail.com>

This file is part of Cight.

Cight is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

Cight is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Cight. If not, see <http://www.gnu.org/licenses/>.
*/

#include <dejavu/averager.h>

#include <clarus/clarus.hpp>
using clarus::List;

namespace dejavu
{

Averager::Averager():
  index_(0),
  size_(0)
{
  // Nothing to do.
}

Averager::Averager(int size):
  index_(0),
  size_(size)
{
  // Nothing to do.
}

cv::Mat Averager::operator () (const cv::Mat &frame)
{
  return next(frame);
}

cv::Mat Averager::operator () (Input input)
{
  return next(input);
}

cv::Mat Averager::next(const cv::Mat &data)
{
  if (average_.empty())
    average_ = cv::Mat(data.size(), data.type(), cv::Scalar::all(0));

  average_ += data;
  if (memory_.size() < size_)
    memory_.append(data);
  else
  {
    cv::Mat &memory = memory_[index_];
    average_ -= memory;
    data.copyTo(memory);
    index_ = (index_ + 1) % size_;
  }

  return average_;
}

cv::Mat Averager::next(Input input)
{
  return next(input());
}

} // namespace dejavu
