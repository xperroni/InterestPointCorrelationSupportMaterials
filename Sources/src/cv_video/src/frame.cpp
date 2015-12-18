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

#include <cv_video/frame.h>
#include <cv_video/video.h>
#include <cv_video/settings.h>

namespace cv_video
{

Frame::Frame()
{
  // Nothing to do.
}

Frame::Frame(const sensor_msgs::ImageConstPtr& message):
  message_(message)
{
  // Nothing to do.
}

Frame::Frame(const sensor_msgs::Image &image)
{
  sensor_msgs::Image *ptr = new sensor_msgs::Image();
  *ptr = image;

  message_.reset(ptr);
}

sensor_msgs::ImageConstPtr Frame::buffer() const
{
  return (copied_.get() != NULL ? copied_->toImageMsg() : message_);
}

cv::Mat Frame::copy()
{
  return copyCvImage()->image;
}

cv_bridge::CvImagePtr Frame::copyCvImage()
{
  if (copied_.get() == NULL)
    copied_ = cv_bridge::toCvCopy(message_, param::encoding());

  return copied_;
}

cv::Mat Frame::share()
{
  return shareCvImage()->image;
}

cv_bridge::CvImageConstPtr Frame::shareCvImage() const
{
  return cv_bridge::toCvShare(message_, param::encoding());
}

uint32_t Frame::index() const
{
  return buffer()->header.seq;
}

void Frame::set(const cv::Mat& image)
{
  cv_bridge::CvImagePtr copied = copyCvImage();
  copied->encoding = encoding(image);
  copied->image = image;
}

} // namespace cv_video