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

#ifndef CV_VIDEO_SUBSCRIBER_H
#define CV_VIDEO_SUBSCRIBER_H

#include <cv_video/video.h>
#include <cv_video/operator.h>

namespace cv_video
{

class Subscriber: public Operator
{
  /** \brief Subscription to image topic. */
  image_transport::Subscriber subscriber_;

  /** \brief Subscription callback. */
  Video::Callback callback_;

  /** \brief Pointer to the parent Video object. */
  Video *video_;

  /**
   * \brief Image topic subscription callback.
   */
  void imageCallback(const sensor_msgs::ImageConstPtr& message);

public:
  /**
   * \brief Create a new subscriber object.
   */
  Subscriber(image_transport::ImageTransport& transport,
             std::string &topic,
             uint32_t queue_size,
             Video::Callback callback,
             Video *video);

  /**
   * \brief Object detructor.
   *
   * Enforces polymorphism. Do not remove.
   */
  virtual ~Subscriber();
};

} // namespace cv_video

#endif
