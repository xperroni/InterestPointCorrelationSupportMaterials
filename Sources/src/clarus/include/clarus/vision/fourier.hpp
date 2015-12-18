/*
Copyright (c) Helio Perroni Filho <xperroni@gmail.com>

This file is part of Clarus.

Clarus is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

Clarus is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Clarus. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef CLARUS_VISION_SNAPSHOT_HPP
#define CLARUS_VISION_SNAPSHOT_HPP

#include <opencv2/opencv.hpp>

namespace fourier
{

cv::Size fit(int width, int height);

cv::Size fit(const cv::Size &size);

cv::Mat convolve(const cv::Mat &data, const cv::Mat &kernel);

/**
\brief Calculates a cross-correlation map between the two matrices.

To compute the cross-correlation, both inputs are normalized by subtracting the
average value from every cell, then dividing by the absolute maximum. Next the
Hadamard (point-wise) product of the Fourier transforms of the normalized matrices
is computed. Finally the inverse Fourier transform of the product is computed.

If <tt>clip = true</tt> (the default) the output matrix is truncated to dimensions
<tt>(data.cols - kernel.cols, data.rows - kernel.rows)</tt>, thus discarding correlation
values that rely on the kernel "going around" the data matrix.

The function requires that <tt>(data.rows > kernel.rows && data.cols > kernel.cols)</tt>
be true. Otherwise a <tt>std::runtime_error</tt> exception is thrown.
*/
cv::Mat correlate(const cv::Mat &data, const cv::Mat &kernel, bool clip = true);

cv::Mat multiply(const cv::Mat &a, const cv::Mat &b, bool conjugate_b = false);

cv::Mat normalize(const cv::Mat &data);

cv::Mat tiles(const cv::Mat &data, int wf);

cv::Mat transform(const cv::Mat &data, int flags = 0, const cv::Size &optimal = cv::Size(0, 0));

cv::Mat inverse(const cv::Mat &fourier, const cv::Size &optimal = cv::Size(0, 0));

cv::Mat magnitude(const cv::Mat &fourier);

cv::Mat phase(const cv::Mat &fourier);

cv::Mat bgr(const cv::Mat &fourier);

/**
 * \brief Rearrange the quadrants of Fourier matrix so the origin is at the center.
 */
cv::Mat swap(const cv::Mat &fourier);

} // namespace fourier

#endif
