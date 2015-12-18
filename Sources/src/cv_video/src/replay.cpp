/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Helio Perroni Filho.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <cv_video/camera_server.h>
#include <cv_video/settings.h>

#include <std_msgs/Bool.h>

namespace cv_video
{

class Replay
{
  /** \brief Reference to the current ROS node. */
  ros::NodeHandle node_;

  /** \brief Subscriber object used to receiver pause / resume commands. */
  ros::Subscriber subscriber_;

  /** \brief Video control object. */
  Video video_;

  /** \brief Path to the replaying video. */
  std::string path_;

  /** \brief Whether the video is currently playing. */
  bool playing_;


  void callback(const std_msgs::BoolConstPtr &message)
  {
    if (message->data == playing_)
      return;

    playing_ = message->data;
    if (playing_)
      video_.resume(path_);
    else
      video_.pause(path_);
  }

public:
  Replay():
    path_(param::path()),
    playing_(param::playing())
  {
    subscriber_ = node_.subscribe<std_msgs::Bool>(name::playing(), 1, &Replay::callback, this);
    video_.replay(path_, name::image(), ros::shutdown);
    if (!playing_)
      video_.pause(path_);
  }
};

void replay(int argc, char** argv)
{
  ros::init(argc, argv, "replay");

  Replay replay;

  ros::spin();
}

} // namespace cv_video

int main(int argc, char** argv)
{
  cv_video::replay(argc, argv);

  return 0;
}
