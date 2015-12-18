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

#ifndef DEJAVU_DIFFERENTIATOR_H
#define DEJAVU_DIFFERENTIATOR_H

#include <dejavu/averager.h>

#include <boost/function.hpp>

#include <opencv2/opencv.hpp>

#include <clarus/clarus.hpp>

namespace dejavu
{

/**
 *
 */
class Differentiator
{
  Averager averager_;

  /** \brief Last received frame that was used to create a difference image. */
  cv::Mat previous_;

  /** \brief Difference image currently under work. */
  cv::Mat differences_;

  /** \brief List of independently updated regions in the difference image. */
  clarus::List<cv::Mat> regions_;

  /** \brief List of rectangles demarcating the independently updated regions. */
  clarus::List<cv::Rect> bounds_;

  /** \brief Average difference threshold for difference images. */
  double threshold_;

  /** \brief Gap between the current "previous" image and the last received image. */
  int gap_;

  /** \brief Maximum acceptable gap. */
  int gap_max_;

public:
  /** \brief Image input function type alias. */
  typedef boost::function<cv::Mat()> Input;

  /**
   * \brief Default constructor.
   */
  Differentiator();

  /**
   * \brief Create a new differentiator of given threshold.
   */
  Differentiator(double threshold);

  /**
   * \brief Functional alias to <tt>next(const cv::Mat &frame)</tt>.
   */
  cv::Mat operator () (const cv::Mat &frame);

  /**
   * \brief Functional alias to <tt>next(Input input)</tt>.
   */
  cv::Mat operator () (Input input);

  /**
   * \brief Try to compute a new difference image.
   *
   * First a difference image between the last used image and the given one is computed.
   * If its average difference trips the threshold, return it. Otherwise return an empty image.
   *
   * If this is the first call, store the given image and return an empty image.
   */
  cv::Mat next(const cv::Mat &frame);

  /**
   * \brief Try to compute a new difference image.
   *
   * Retrieve images from the input function until a difference image that trips the the threshold
   * is computed. If the input function returns an empty image, return an empty image.
   */
  cv::Mat next(Input input);
};

} // namespace dejavu

#endif
