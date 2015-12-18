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

#include <dejavu/file_stream.h>

namespace dejavu
{

FileStream::FileStream():
  input_(""),
  sampling_(0)
{
  // Nothing to do.
}

FileStream::FileStream(int sampling, const std::string &path):
  input_(path),
  sampling_(sampling)
{
  // Nothing to do.
}

cv::Mat FileStream::operator () ()
{
  return next();
}

cv::Mat FileStream::next()
{
  for (int i = 0; i < sampling_; i++)
    input_();

  return input_();
}

} // namespace dejavu
