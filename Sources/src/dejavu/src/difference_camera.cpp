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

#include <dejavu/differentiator.h>
#include <dejavu/settings.h>

#include <cv_video/video.h>
using cv_video::Video;
using cv_video::Frame;

#include <ros/console.h>

namespace dejavu
{

class DifferenceCamera
{
  std::string topic_;

  Differentiator differentiator_;

  int index_;

  void update(Video &video, Frame &frame)
  {
    cv::Mat diff = differentiator_(frame.copy());
    if (diff.empty())
      return;

    ROS_INFO_STREAM("Input indexes: (" << index_++ << ", " << frame.index() << ")");

    frame.set(diff);
    video.publish(topic_, frame);
  }

public:
  DifferenceCamera():
    topic_(name::image_difference()),
    differentiator_(param::difference_threshold()),
    index_(0)
  {
    // Nothing to do.
  }

  void spin()
  {
    Video video;
    video.subscribe("difference_camera", &DifferenceCamera::update, this);
    ros::spin();
  }
};

} // namespace dejavu

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "difference_camera");

  dejavu::DifferenceCamera camera;

  camera.spin();

  return 0;
}
