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

#ifndef CV_VIDEO_FRAME_H
#define CV_VIDEO_FRAME_H

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.h>

namespace cv_video
{

class Frame
{
  /** \brief ROS image message. */
  sensor_msgs::ImageConstPtr message_;

  /** \brief Copied CvImage. */
  cv_bridge::CvImagePtr copied_;

public:
  /**
   * \brief Default constructor.
   */
  Frame();

  /**
   * \brief Create a new frame from the given ROS image message.
   */
  Frame(const sensor_msgs::ImageConstPtr& message);

  /**
   * \brief Create a new frame from the given ROS image message.
   */
  Frame(const sensor_msgs::Image &image);

  /**
   * \brief Return a reference to the latest frame buffer.
   */
  sensor_msgs::ImageConstPtr buffer() const;

  /**
   * \brief Return a copied (modifiable) OpenCV image.
   *
   * Changes to the pointed object will be preserved in this frame.
   */
  cv::Mat copy();

  /**
   * \brief Return a copied (modifiable) CvImage.
   *
   * Changes to the pointed object will be preserved in this frame.
   */
  cv_bridge::CvImagePtr copyCvImage();

  /**
   * \brief Return a shared (non-modifiable) OpenCV image.
   */
  cv::Mat share();

  /**
   * \brief Return a shared (non-modifiable) CvImage.
   */
  cv_bridge::CvImageConstPtr shareCvImage() const;

  /**
   * \brief Return the sequential ID of the enclosed image.
   */
  uint32_t index() const;

  /**
   * \brief Update the image buffer of the copied CvImage enclosed in this object.
   */
  void set(const cv::Mat& image);
};

} // namespace cv_video

#endif
