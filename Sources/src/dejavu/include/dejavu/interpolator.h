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

#ifndef DEJAVU_INTERPOLATOR_H
#define DEJAVU_INTERPOLATOR_H

#include <clarus/clarus.hpp>

#include <opencv2/opencv.hpp>

namespace dejavu
{

cv::Point lineP0(const cv::Point2f &line);

cv::Point linePn(const cv::Point2f &line, const cv::Size &size);

cv::Point interpolateParameters(cv::Mat &parameters, float bias, const cv::Mat &similarities);

cv::Point2f interpolateParameters(cv::Mat &parameters, float bias, float x, float y0, const cv::Mat &similarities);

cv::Point interpolateEnumerate(const cv::Mat &similarities);

cv::Point2f interpolateFit(const clarus::List<cv::Point2f> &points, double m0 = 0.0, double mN = 15.0);

cv::Point2f interpolateBest(const clarus::List<cv::Point2f> &points, double d_min, double error = 1.0);

} // namespace dejavu

#endif
