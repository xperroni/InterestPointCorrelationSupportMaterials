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
#include <dejavu/feature_selector.h>
#include <dejavu/interpolator.h>
#include <dejavu/settings.h>
#include <dejavu/working_memory.h>

#include <cv_video/settings.h>

#include <cv_video/video.h>
using cv_video::Video;
using cv_video::Frame;

#include <clarus/clarus.hpp>
using clarus::List;

#include <ros/console.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>

#include <ctime>

namespace dejavu
{

class DifferenceMatcher: Dispatcher
{
  struct Teach
  {
    List<cv::Mat> images;

    int index;

    int padding;

    Teach():
      index(0),
      padding(param::teach_padding())
    {
      // Nothing to do.
    }

    void bootstrap()
    {
      ROS_INFO_STREAM("Teach records bootstrap");

      DifferenceStream stream(param::difference_threshold(), param::path());
      for (int i = 0;; i++)
      {
        cv::Mat diff = stream();
        if (diff.empty())
          break;

        images.append(diff);

        ROS_INFO_STREAM("Teach indexes: (" << i << ", " << stream.index << ")");
      }

      ROS_INFO_STREAM("Teach records bootstrap done");
    }

    void shift(int count)
    {
      index += count;
    }

    bool done()
    {
      return (index + param::teach_images() >= images.size());
    }
  };

  struct Replay
  {
    List<cv::Mat> images;

    int index;

    int padding;

    Replay():
      index(0),
      padding(param::replay_padding())
    {
      // Nothing to do.
    }

    void append(Frame &frame)
    {
      index = frame.index();
      images.append(frame.copy());
    }
  };

  struct Similarities: public cv::Mat
  {
    Teach teach;

    Replay replay;

    cv::Mat parameters;

    geometry_msgs::Point matching;

    Similarities():
      cv::Mat(param::teach_images(), param::replay_images(), CV_32F, cv::Scalar(0))
    {
      // Nothing to do.
    }

    bool accumulate(Frame &frame)
    {
      ROS_INFO_STREAM("Save up frame #" << frame.index());

      replay.append(frame);
      return (replay.images.size() >= cols);
    }

    void bootstrap()
    {
      teach.bootstrap();

      ROS_INFO_STREAM("Similarity matrix bootstrap");

      for (int j = 0; j < cols; j++)
        set(j, replay.images[j]);

      replay.images.clear();

      update();

      ROS_INFO_STREAM("Similarity matrix bootstrap done");
    }

    void set(int j, const cv::Mat &diff)
    {
      clock_t t = clock();

      List<FeaturePoint> features = selectSparseDifferences(30, diff, replay.padding, teach.padding);
      for (int k = 0, m = features.size(); k < m; k++)
      {
        const FeaturePoint &point = features[k];
        int i = point(teach.images, teach.index, rows);
        at<float>(i, j) += 1;
      }

      ROS_INFO_STREAM("updateColumn time: " << ((double) (clock() - t)) / ((double) CLOCKS_PER_SEC) << "s");
    }

    void set(Frame &frame)
    {
      int col_skip = frame.index() - replay.index;
      int row_skip = col_skip; // TODO: calculate row skip

      replay.index = frame.index();

      teach.shift(row_skip);

      clarus::shift(*this, -row_skip, -col_skip);
      set(cols - 1, frame.share());

      update();

      ROS_INFO_STREAM("Frame skip: (" << row_skip << ", " << col_skip << ")");
    }

    void update()
    {
      cv::Point line = interpolateParameters(parameters, 0.75, *this);
      matching.x = line.x;
      matching.y = line.y;

      ROS_INFO_STREAM("Matching: (" << matching.x << ", " << matching.y << ")");
    }
  };

public:
  typedef void(DifferenceMatcher::*State)(Video&, Frame&);

  Similarities similarities;

  Video visual;

  DifferenceMatcher():
    similarities(),
    visual(name::image_difference())
  {
    set(&DifferenceMatcher::fill);
  }

  void fill(Video &video, Frame &frame)
  {
    if (similarities.accumulate(frame))
    {
      set(&DifferenceMatcher::update);

      publish<bool>(name::forward(), false, true);

      similarities.bootstrap();

      publish<bool>(name::forward(), true, true);
    }
  }

  void update(Video &video, Frame &frame)
  {
    try
    {
      similarities.set(frame);
      if (similarities.teach.done())
      {
        publish<bool>(name::forward(), false, true);
        ros::shutdown();
      }

      publish<geometry_msgs::Point>(name::matching(), similarities.matching);

      visual.publish(name::image_matching(), depths::bgr(similarities), true);
    }
    catch (...)
    {
      ROS_ERROR_STREAM("update() unknown error");
      ros::shutdown();
    }
  }

  void set(State state)
  {
    Video::Callback callback = boost::bind(state, this, _1, _2);
    //visual.subscribe("input", replay.images.limit(), callback);
    visual.subscribe("input", callback);
  }
};

void difference_matcher(int argc, char *argv[])
{
  ros::init(argc, argv, "difference_matcher");

  DifferenceMatcher matcher;

  ros::spin();
}

} // namespace dejavu

int main(int argc, char *argv[])
{
  dejavu::difference_matcher(argc, argv);

  return 0;
}