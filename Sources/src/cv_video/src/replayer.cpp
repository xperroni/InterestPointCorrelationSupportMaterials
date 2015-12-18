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

#include <cv_video/replayer.h>

#include <cv_video/settings.h>

namespace cv_video
{

Replayer::Replayer(const std::string &path, Publisher::Ptr publisher, double fps, Callback callback)
{
  start(path, publisher, fps, callback);
}

Replayer::Replayer(const std::string &path, Publisher::Ptr publisher, Callback callback)
{
  start(path, publisher, 0, callback);
}

Replayer::Replayer(const std::string &path, Callback callback)
{
  start(path, callback);
}

Replayer::~Replayer()
{
  // Nothing to do.
}

void Replayer::start(const std::string &path, Publisher::Ptr publisher, double fps, Callback callback)
{
  start(path, callback);
  publisher_ = publisher;

  ros::NodeHandle node;
  ros::Duration period(1.0 / (fps != 0 ? fps : param::fps()));
  timer_ = node.createTimer(period, &Replayer::timerCallback, this);
}

void Replayer::start(const std::string &path, Callback callback)
{
  done_ = false;
  callback_ = callback;
  video_.reset(new cv::VideoCapture(path));
}

void Replayer::timerCallback(const ros::TimerEvent& event)
{
  cv::Mat frame;
  if (video_->read(frame))
    publisher_->publish(frame);
  else
    stop();
}

cv::Mat Replayer::operator () ()
{
  return next();
}

bool Replayer::done()
{
  return done_;
}

cv::Mat Replayer::next()
{
  if (done_)
    return cv::Mat();

  cv::Mat frame;
  if (publisher_.get() != NULL)
  {
    video_->retrieve(frame, 0);
    return frame;
  }

  if (video_->read(frame))
    return frame;

  stop();
  return cv::Mat();
}

void Replayer::pause()
{
  timer_.stop();
}

void Replayer::resume()
{
  timer_.start();
}

void Replayer::stop()
{
  done_ = true;
  video_.reset();

  if (timer_.isValid())
    timer_.stop();

  if (callback_)
    callback_();
}

} // namespace cv_video
