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

#ifndef DEJAVU_FEATURE_SELECTOR
#define DEJAVU_FEATURE_SELECTOR

#include <dejavu/feature_point.h>

#include <clarus/core/list.hpp>

#include <opencv2/opencv.hpp>

namespace dejavu
{

clarus::List<FeaturePoint> selectSparseDifferences(int limit, const cv::Mat &image, int padding, int padding2);

clarus::List<FeaturePoint> selectSparseSaliences(int limit, const cv::Mat &image, int padding, int padding2);

clarus::List<FeaturePoint> selectSparseIntegrals(int limit,
                                                 const cv::Mat &image,
                                                 const cv::Mat &integral,
                                                 int padding,
                                                 int padding2);

} // namespace dejavu

#endif
