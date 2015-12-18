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

#include <dejavu/feature_map.h>
using clarus::List;

#include <clarus/core/math.hpp>

#include <map>

namespace dejavu
{

FeatureMap::FeatureMap(Selector selector, const cv::Mat &bgr, int padding):
  features_(selector(bgr, padding))
{
  // Nothing to do.
}

FeatureMap::FeatureMap(const List<FeaturePoint> &features):
  features_(features)
{
  // Nothing to do.
}

cv::Mat FeatureMap::operator () (const List<cv::Mat> &images, int padding, int i0, int n) const
{
  int rows = (n == 0 ? images.size() : n);
  int cols = features_.size();

  cv::Mat responses(rows, 1, CV_32F, cv::Scalar(0));
  for (int i = 0; i < cols; i++)
  {
    const FeaturePoint &point = features_[i];
    int index = point(images, i0, rows, padding);
    responses.at<float>(index, 0) += 1.0f;
  }

  return responses;
}

size_t FeatureMap::size() const
{
  return features_.size();
}

} // namespace dejavu
