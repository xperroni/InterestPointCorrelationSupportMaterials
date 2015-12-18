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

#include <dejavu/shift_reduce.h>

namespace dejavu
{

reduce_slip::reduce_slip():
    last_(INT_MIN)
{
    // Nothing to do.
}

inline double drag(int y, int i, double g)
{
  double d = fabs(y - i);
  return (d == 0 ? g : g / d);
}

inline cv::Mat normalize(const cv::Mat &responses)
{
  double minVal = 0.0;
  cv::minMaxLoc(responses, &minVal);
  cv::Mat normal = responses - minVal;

  double maxVal = 0.0;
  cv::minMaxLoc(responses, NULL, &maxVal);
  normal /= maxVal;

  return normal;
}

inline int correction(int y, const cv::Mat &responses)
{
  int cols = responses.cols;

  int yl = y;
  double rl = responses.at<double>(0, y);
  for (int i = y - 1; i >= 0; i--)
  {
    double r = responses.at<double>(0, i);
    if (r < rl)
      break;

    if (r > rl)
    {
      yl = i;
      rl = r;
    }
  }

  int yr = y;
  double rr = responses.at<double>(0, y);
  for (int i = y + 1; i < cols; i++)
  {
    double r = responses.at<double>(0, i);
    if (r < rr)
      break;

    if (r > rr)
    {
      yr = i;
      rr = r;
    }
  }

  if (drag(y, yl, rl) > drag(y, yr, rr))
    return yl;
  else
    return yr;
}

int reduce_slip::operator () (const cv::Mat &responses)
{
  if (last_ != INT_MIN)
    last_ = correction(last_, normalize(responses));
  else
    last_ = reduce_wta(responses) + responses.cols / 2;

  return last_ - responses.cols / 2;
}

int reduce_wta(const cv::Mat &responses)
{
  cv::Point maxLoc;
  cv::minMaxLoc(responses, NULL, NULL, NULL, &maxLoc);
  return maxLoc.x - responses.cols / 2;
}

} // namespace dejavu
