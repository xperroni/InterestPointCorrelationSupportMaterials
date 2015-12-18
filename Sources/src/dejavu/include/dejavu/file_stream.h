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

#ifndef DEJAVU_FILE_STREAM_H
#define DEJAVU_FILE_STREAM_H

#include <cv_video/replayer.h>

namespace dejavu
{

/**
 *
 */
class FileStream
{
  /** \brief Video record. */
  cv_video::Replayer input_;

  /** \brief Distance between returned frames. */
  int sampling_;

public:
  /**
   * \brief Default constructor.
   */
  FileStream();

  /**
   * \brief Create a new file stream of given sampling for the given video file.
   */
  FileStream(int sampling, const std::string &path);

  /**
   * \brief Functional alias to <tt>next()</tt>.
   */
  cv::Mat operator () ();

  /**
   * \brief Return a new video frame.
   *
   * If all frames have already been used, return an empty image.
   */
  cv::Mat next();
};

} // namespace dejavu

#endif
