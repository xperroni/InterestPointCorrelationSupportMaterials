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

#ifndef DEJAVU_AVERAGER_H
#define DEJAVU_AVERAGER_H

#include <boost/function.hpp>

#include <opencv2/opencv.hpp>

#include <clarus/clarus.hpp>

namespace dejavu
{

/**
 *
 */
class Averager
{
  /** \brief Most recent received frames. */
  clarus::List<cv::Mat> memory_;

  /** \brief Index of the next position to update. */
  int index_;

  /** \brief Number of elements used in the average. */
  int size_;

  /** \brief Running average. */
  cv::Mat average_;

public:
  /** \brief Image input function type alias. */
  typedef boost::function<cv::Mat()> Input;

  /**
   * \brief Default constructor.
   */
  Averager();

  /**
   * \brief Create a new sliding average function of given size.
   */
  Averager(int size);

  /**
   * \brief Functional alias to <tt>next(const cv::Mat &frame)</tt>.
   */
  cv::Mat operator () (const cv::Mat &data);

  /**
   * \brief Functional alias to <tt>next(Input input)</tt>.
   */
  cv::Mat operator () (Input input);

  /**
   * \brief Compute a new sliding average.
   */
  cv::Mat next(const cv::Mat &data);

  /**
   * \brief Compute a new sliding average.
   */
  cv::Mat next(Input input);
};

} // namespace dejavu

#endif
