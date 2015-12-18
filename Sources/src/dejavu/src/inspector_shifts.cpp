/*
 * Copyright (c) Helio Perroni Filho <xperroni@gmail.com>
 * 
 * This file is part of Dejavu.
 * 
 * Dejavu is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * Dejavu is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with Dejavu. If not, see <http://www.gnu.org/licenses/>.
 */

#include <dejavu/difference_stream.h>
#include <dejavu/filters.h>
#include <dejavu/settings.h>
#include <dejavu/working_memory.h>

#include <cv_video/video.h>
using cv_video::Video;
using cv_video::Frame;

#include <clarus/core/math.hpp>
#include <clarus/io/viewer.hpp>
#include <clarus/vision/depths.hpp>
#include <clarus/vision/fourier.hpp>

#include <ros/console.h>
#include <geometry_msgs/Point.h>

namespace dejavu
{

class ShiftInspector
{
  struct Teach
  {
    DifferenceStream stream;

    WorkingMemory<cv::Mat> memory;

    /** \brief Index of the first teach image in the working memory. */
    int offset;

    /** \brief Teach / replay image matching parameters. */
    geometry_msgs::Point matching;

    Teach():
      stream(param::difference_threshold(), param::path_teach()),
      memory(param::teach_images()),
      offset(0)
    {
      while (memory.idle())
        memory.append(stream());

      matching.x = 0.0;
      matching.y = 0.0;
      matching.z = 0.96; // Common result
    }

    cv::Mat images(int index)
    {
      int i = matching.y + (index - matching.x) * matching.z - offset;
      while (i >= memory.size())
      {
        cv::Mat diff = stream();
        if (diff.empty())
          return cv::Mat();

        memory.append(diff);
        offset++;
        i--;
      }

      return memory[i];
    }
  };

  class Replay
  {
    DifferenceStream input_;

    /** \brief Width of the sliding window used in the shift vector computation. */
    int slit_;

    /** \brief Leaky integrator constant. */
    double leak_;

    /** \brief Shift likelihood vector. */
    cv::Mat shifts_;

  public:
    /** \brief Index of the last received difference image. */
    int index;

    Replay():
      input_(param::difference_threshold(), param::path_replay()),
      slit_(param::divs_width()),
      leak_(param::replay_leak()),
      index(0)
    {
      // Discard images used for difference matching.
      for (index = 0; index < param::replay_images(); index++)
        input_();
    }

    void showVector(const std::string &title, const cv::Mat &vector)
    {
      static cv::Scalar BLACK(0, 0, 0);
      static cv::Scalar WHITE(255, 255, 255);

      int cols = vector.cols;
      cv::Mat bgr = depths::bgr(vector, cv::Size(cols, 100));
      cv::Point point = clarus::argmax(vector);

      bgr(cv::Rect(cols / 2, 10, 1, 80)) = BLACK;
      bgr(cv::Rect(point.x, 10, 1, 80)) = WHITE;

//      cv::Mat peak(bgr, cv::Rect(point.x, 10, 1, 80));
//      peak = WHITE;

      viewer::show(title, bgr);
    }

    bool operator () (Teach &teacher)
    {
      cv::Mat replay = input_();
      cv::Mat teach = teacher.images(index);
      if (teach.empty())
        return false;

      cv::Mat teachMap = upperHalf(teach);
      cv::Mat replayMap = upperHalf(replay);

      int rows = teachMap.rows;
      int cols = teachMap.cols;

      int slide = cols - slit_;

      bool skip = false;
      cv::Mat responses(1, slide * 2, CV_64F, cv::Scalar::all(0));
      for (int i = 0; i <= slide && ros::ok(); i+= slit_) {
          cv::Mat slice(teachMap, cv::Rect(i, 0, slit_, rows));
          cv::Mat correlated = fourier::correlate(replayMap, slice);
          //cv::Mat shifts(correlated, cv::Rect(0, 0, slide, 1));

          cv::Mat range(responses, cv::Rect(slide - i, 0, slide, 1));
          range += correlated;

          viewer::show("Teach map", depths::bgr(teachMap));
          viewer::show("Replay map", depths::bgr(replayMap));
          viewer::show("Teach slice", depths::bgr(slice));
          showVector("Correlation", correlated);

          if (skip)
            continue;

          int key = cv::waitKey();
          if (key == 'n')
            skip = true;
          if (key == 'q')
            return false;
      }

      if (shifts_.empty())
        shifts_ = cv::Mat(1, slide * 2, CV_64F, cv::Scalar::all(0));

      shifts_ = leak_ * shifts_ + (1.0 - leak_) * responses;

      showVector("Responses", responses);
      showVector("Shifts", shifts_);
      std::cerr << index - param::replay_images() << std::endl;

      index++;

      return true;
    }
  };

public:
  Teach teach;

  Replay replay;

  ShiftInspector()
  {
    while(replay(teach) && ros::ok())
    {
      // Nothing to do.
    }
  }
};

} // namespace dejavu

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "shift_inspector");

  dejavu::ShiftInspector shift_inspector;

  return 0;
}
