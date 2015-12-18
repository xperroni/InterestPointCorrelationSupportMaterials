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

#include <dejavu/difference_stream.h>

namespace dejavu
{

DifferenceStream::DifferenceStream():
  differentiator_(0.0),
  input_(""),
  index(-1)
{
  // Nothing to do.
}

DifferenceStream::DifferenceStream(double threshold, const std::string &path):
  differentiator_(threshold),
  input_(path),
  index(-1)
{
  // Nothing to do.
}

cv::Mat DifferenceStream::operator () ()
{
  return next();
}

cv::Mat DifferenceStream::next()
{
  for (;;)
  {
    index++;

    cv::Mat image = input_();
    if (image.empty())
      break;

    cv::Mat diff = differentiator_(image);
    if (!diff.empty())
      return diff;
  }

  return cv::Mat();
}

} // namespace dejavu
