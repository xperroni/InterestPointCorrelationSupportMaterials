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

#ifndef DEJAVU_FEATURE_POINT_H
#define DEJAVU_FEATURE_POINT_H

#include <clarus/clarus.hpp>

#include <opencv2/opencv.hpp>

namespace dejavu
{

struct FeaturePoint
{
  /** \brief Bounds of the patch around the feature point. */
  cv::Rect bounds_patch;

  /** \brief Bounds of the search area. */
  cv::Rect bounds_search;

  /** \brief Bounds of the reach area. */
  cv::Rect bounds_reach;

  /** \brief Feature point's coordinates. */
  cv::Point center;

  /** \brief Contents of the patch around the feature point. */
  cv::Mat patch;

  /** \brief Search area for the patch. */
  cv::Size reach;

  /** \brief Fourier transform of the patch. */
  cv::Mat spectra;

  /** \brief The feature point's relevance value. */
  float strength;

  /**
  \brief Creates a new, blank feature point.
  */
  FeaturePoint();

  /**
  \brief Creates a new feature point at or close to the given coordinates.

  A patch of side <tt>2 * padding + 1</tt> is extracted from the given image and
  associated to the feature point. The feature point's coordinates may be moved towards
  the center of the image, if part of the patch would otherwise fall outside image
  borders.
  */
  FeaturePoint(int x, int y, const cv::Mat &image, int padding, int padding2 = 0);

  int operator () (const clarus::List<cv::Mat> &images, int i0, int n) const;
};

} // namespace dejavu

#endif
