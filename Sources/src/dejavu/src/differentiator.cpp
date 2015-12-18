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

#include <dejavu/differentiator.h>

#include <dejavu/filters.h>

#include <clarus/clarus.hpp>
using clarus::List;

namespace dejavu
{

Differentiator::Differentiator():
  averager_(6),
  threshold_(0.0),
  gap_(0),
  gap_max_(6)
{
  // Nothing to do.
}

Differentiator::Differentiator(double threshold):
  averager_(5),
  threshold_(threshold),
  gap_(0),
  gap_max_(5)
{
  int width = 640;
  int width_side = width / 4;
  int height = 240;

  cv::Rect left(0, 0, width_side, height);
  cv::Rect right(width - width_side, 0, width_side, height);
  cv::Rect top(left.width, 0, 2 * width_side, height / 2);
  cv::Rect center(left.width, top.height, width / 2, height / 2);

  bounds_.append(left);
  bounds_.append(right);
  bounds_.append(top);
  bounds_.append(center);
}

cv::Mat Differentiator::operator () (const cv::Mat &frame)
{
  return next(frame);
}

cv::Mat Differentiator::operator () (Input input)
{
  return next(input);
}

cv::Mat Differentiator::next(const cv::Mat &image)
{
/*
  if (image.empty())
    return cv::Mat();

  cv::Mat saliences = averager_(saliencyMap(image, 64));
  if (++gap_ < gap_max_)
    return cv::Mat();

  gap_ = 0;
  return colors::discrete(saliences);
*/
/*
  if (++gap_ >= gap_max_)
  {
    gap_ = 0;
    return colors::discrete(saliences);
  }

  return cv::Mat();
*/
/*
  if (previous_.empty())
  {
    previous_ = image;
    differences_ = cv::Mat(image.size(), CV_8U, cv::Scalar::all(0));
    for (int i = 0, n = bounds_.size(); i < n; i++)
      regions_.append(differences_(bounds_[i]));

    return cv::Mat();
  }

  bool incomplete = false;
  for (int i = 0, n = bounds_.size(); i < n; i++)
  {
    if (clarus::mean(regions_[i]) >= threshold_)
      continue;

    cv::Rect &roi = bounds_[i];
    cv::Mat p(previous_, roi);
    cv::Mat q(image, roi);

    cv::Mat d = images::difference(p, q, CV_8U);
    double mean = clarus::mean(d);
    if (mean < threshold_)
        incomplete = true;

    d.copyTo(regions_[i]);
  }

  if (incomplete && gap_++ < gap_max_)
    return cv::Mat();

  cv::Mat out = differences_.clone();
  differences_ = cv::Scalar(0);
  previous_ = image;
  gap_ = 0;

  return out;
*/

  if (previous_.empty())
  {
    previous_ = image;
    return cv::Mat();
  }

  cv::Mat diffs = images::difference(previous_, image, CV_64F);
  if (clarus::mean(diffs) < threshold_)
    return cv::Mat();

  previous_ = image;
  return diffs;

}

cv::Mat Differentiator::next(Input input)
{
  for (;;)
  {
    cv::Mat image = input();
    if (image.empty())
      return cv::Mat();

    cv::Mat diff = next(image);
    if (!diff.empty())
      return diff;
  }
}

} // namespace dejavu
