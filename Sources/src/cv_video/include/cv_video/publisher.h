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

#ifndef CV_VIDEO_PUBLISHER_H
#define CV_VIDEO_PUBLISHER_H

#include <cv_video/frame.h>
#include <cv_video/operator.h>

#include <cv_bridge/cv_bridge.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <boost/shared_ptr.hpp>

namespace cv_video
{

class Publisher: public Operator
{
  /** \brief Image publisher. */
  image_transport::Publisher publisher_;

  /** \brief Incrementing message ID. */
  uint32_t index_;

public:
  /** \brief Smart pointer type alias. */
  typedef boost::shared_ptr<Publisher> Ptr;

  /**
   * \brief Default constructor.
   */
  Publisher();

  /**
   * \brief Create a new publisher connected to the given topic.
   *
   * If the \c latch argument is \c true, published images are latched to the destination topic.
   */
  Publisher(image_transport::ImageTransport& transport, std::string topic, bool latch = false);

  /**
   * \brief Object destructor.
   *
   * Enforces polymorphism. Do not remove.
   */
  virtual ~Publisher();

  /**
   * \brief Publish the given image.
   */
  void publish(const cv::Mat& image);

  /**
   * \brief Publish the given image.
   */
  void publish(const cv_bridge::CvImagePtr& image);

  /**
   * \brief Publish the given image.
   */
  void publish(const cv_bridge::CvImageConstPtr& image);

  /**
   * \brief Publish the given image.
   */
  void publish(const Frame& frame);
};

} //namespace cv_video

#endif
