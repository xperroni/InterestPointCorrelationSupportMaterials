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

#ifndef DEJAVU_DIFFERENCE_STREAM_H
#define DEJAVU_DIFFERENCE_STREAM_H

#include <dejavu/differentiator.h>

#include <cv_video/replayer.h>

namespace dejavu
{

/**
 *
 */
class DifferenceStream
{
  /** \brief Differentiator object used to compute difference images. */
  Differentiator differentiator_;

  /** \brief Video record. */
  cv_video::Replayer input_;

public:
  int index;

  /**
   * \brief Default constructor.
   */
  DifferenceStream();

  /**
   * \brief Create a new difference stream of given threshold for the given video file.
   */
  DifferenceStream(double threshold, const std::string &path);

  /**
   * \brief Functional alias to <tt>next()</tt>.
   */
  cv::Mat operator () ();

  /**
   * \brief Try to compute a new difference image.
   *
   * If all video frames have already been used, return an empty image.
   */
  cv::Mat next();
};

} // namespace dejavu

#endif
