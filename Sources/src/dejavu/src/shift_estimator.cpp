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
#include <dejavu/dispatcher.h>
#include <dejavu/filters.h>
#include <dejavu/settings.h>
#include <dejavu/shift_reduce.h>
#include <dejavu/working_memory.h>

#include <cv_video/video.h>
using cv_video::Video;
using cv_video::Frame;

#include <clarus/clarus.hpp>
using clarus::List;

#include <ros/console.h>
#include <geometry_msgs/Point.h>

namespace dejavu
{

class ShiftEstimator: Dispatcher
{
  class Teach
  {
    List<cv::Mat> images_;

  public:
    /** \brief Index of the last returned difference image. */
    int index;

    /** \brief Teach / replay image matching parameters. */
    geometry_msgs::Point matching;

    Teach()
    {
      ROS_INFO_STREAM("Teach records bootstrap");

      for (DifferenceStream stream(param::difference_threshold(), param::path());;)
      {
        cv::Mat diff = stream();
        if (diff.empty())
          break;

        images_.append(diff);
      }

      ROS_INFO_STREAM("Teach records bootstrap done (size: " << images_.size() << ")");

      matching.x = 0.0;
      matching.y = 0.0;
      matching.z = DBL_MAX;
    }

    cv::Mat images(int j)
    {
      if (matching.z == DBL_MAX)
        return cv::Mat();

      int k = matching.y + (j - matching.x) * matching.z;
      if (k < 0)
        return images_[0];
      if (k >= images_.size())
        return images_[-1];

      index = k;

      return images_[k];
    }
  };

  class Replay
  {
    /** \brief First index from which to publish shift vectors. */
    int index0_;

    /** \brief Width of the sliding window used in the shift vector computation. */
    int slit_;

    /** \brief Leaky integrator constant. */
    double leak_;

    WorkingMemory<cv::Mat> correlations_;

    /** \brief Shift likelihood vector. */
    cv::Mat shifts_;

  public:
    /** \brief Index of the last received difference image. */
    int index;

    Replay():
      index0_(2 * param::replay_images()),
      slit_(param::divs_width()),
      leak_(param::replay_leak()),
      correlations_(25),
      index(0)
    {
      // Nothing to do.
    }

    cv::Mat operator () (Frame &frame, Teach &teacher)
    {
      index = frame.index();
      cv::Mat replay = frame.share();
      cv::Mat teach = teacher.images(index);
      if (teach.empty())
        return cv::Mat();

      cv::Mat teachMap = upperHalf(teach);
      cv::Mat replayMap = upperHalf(replay);

      int rows = teachMap.rows;
      int cols = teachMap.cols;

      int slide = cols - slit_;

      cv::Mat responses(1, slide * 2, CV_64F, cv::Scalar::all(0));
      for (int i = 0; i <= slide; i+= slit_) {
          cv::Mat slice(teachMap, cv::Rect(i, 0, slit_, rows));
          cv::Mat correlated = fourier::correlate(replayMap, slice);
          cv::Mat shifts(correlated, cv::Rect(0, 0, slide, 1));

          cv::Mat range(responses, cv::Rect(slide - i, 0, slide, 1));
          range += shifts;
      }

      ROS_INFO_STREAM("Correlation: (" << index << ", " << responses << ")");

      if (shifts_.empty())
        shifts_ = cv::Mat(1, cols, CV_64F, cv::Scalar::all(0));

      cv::Mat correlation(responses, cv::Rect(slide - cols / 2, 0, cols, 1));

      if (!correlations_.idle())
        shifts_ -= correlations_[0];

      correlations_.append(correlation);
      shifts_ += correlation;

      ROS_INFO_STREAM("Shifts: (" << index << ", " << shifts_ << ")");

      return (index < index0_ ? cv::Mat() : shifts_);
    }
  };

  /** \brief Topic to which the direction is published. */
  std::string topic_;

  Video video_;

  void upadteMatching(const geometry_msgs::PointConstPtr &matching)
  {
    teach.matching = *matching;
  }

  void updateShift(Video &video, Frame &frame)
  {
    cv::Mat shift = replay(frame, teach);
    if (shift.empty())
      return;

    int direction = reduce_wta(shift);
    publish<int>(topic_, direction);

    ROS_INFO_STREAM("Match: (" << replay.index << ", " << teach.index << ", " << direction << ")");
  }

public:
  Teach teach;

  Replay replay;

  ShiftEstimator():
    topic_(name::direction()),
    video_(name::image_difference())
  {
    subscribe<geometry_msgs::Point>(name::matching(), &ShiftEstimator::upadteMatching, this);
    video_.subscribe("shift_estimator", &ShiftEstimator::updateShift, this);
  }
};

} // namespace dejavu

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "shift_estimator");

  dejavu::ShiftEstimator shift_estimator;

  ros::spin();

  return 0;
}
