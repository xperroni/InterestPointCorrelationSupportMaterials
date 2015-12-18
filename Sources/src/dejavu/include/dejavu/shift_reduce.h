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

#ifndef DEJAVU_SHIFT_REDUCE_H
#define DEJAVU_SHIFT_REDUCE_H

#include <opencv2/opencv.hpp>
#include <boost/function.hpp>

namespace dejavu
{

int reduce_wta(const cv::Mat &responses);

class reduce_slip
{
  int last_;

public:
  reduce_slip();

  int operator () (const cv::Mat &responses);
};

} // namespace dejavu

#endif
