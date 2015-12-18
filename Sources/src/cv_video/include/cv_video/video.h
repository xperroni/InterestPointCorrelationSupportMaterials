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

#ifndef CV_VIDEO_VIDEO_H
#define CV_VIDEO_VIDEO_H

#include <cv_video/frame.h>
#include <cv_video/publisher.h>
#include <cv_video/Record.h>

#include <cv_bridge/cv_bridge.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <boost/shared_ptr.hpp>

#include <opencv2/opencv.hpp>

#include <string>

namespace cv_video
{

class Video
{
  /** \brief Reference to the current ROS node. */
  ros::NodeHandle node_;

  /** \brief Image topic to listen to. */
  std::string topic_;

  /** \brief Access to the image channel. */
  image_transport::ImageTransport transport_;

  /** \brief Image topic operators. */
  std::map<std::string, Operator::Ptr> operators_;

public:
  /** \brief Smart pointer type alias. */
  typedef boost::shared_ptr<Video> Ptr;
  
  /** \brief Callback function type. */
  typedef boost::function<void(Video&, Frame&)> Callback;

  /** \brief Callback type for signaling a video replay has finished. */
  typedef boost::function<void()> Finished;

  /** \brief Create a new object bound to the default \c image topic. */
  Video();
  
  /** \brief Create a new object bound to the given topic. */
  Video(const std::string& topic);
  
  /**
   * \brief Publish the given image to the given image topic.
   */
  template<class T>
  void publish(const std::string& topic, const T& image, bool latch = false);
  
  /**
   * \brief Removes the publisher for the given topic.
   */
  void unpublish(const std::string& topic);

  /**
   * \brief Record video to the default file path.
   */
  void record();

  /**
   * \brief Record video to the given file.
   */
  void record(const std::string& path);

  /**
   * \brief Record video to the given file, using the given parameters.
   */
  void record(const std::string& path,
              const std::string& format,
              double fps,
              int width,
              int height);

  /**
   * \brief Record video using the given parameters.
   */
  void record(const Record& request);

  /**
   * \brief Replays the given video file at the given frame rate.
   *
   * \arg path Path to the video file to replay.
   *
   * \arg topic Topic at which the recorded video is to be replayed.
   *
   * \arg Frame rate of the video. If omitted, the current default rate is used.
   */
  void replay(const std::string &path, const std::string &topic, double fps = 0);

  /**
   * \brief Replays the given video file at the default frame rate.
   *
   * \arg path Path to the video file to replay.
   *
   * \arg topic Topic at which the recorded video is to be replayed.
   *
   * \arg finished Function to be called when the video finishes playing.
   */
  void replay(const std::string &path, const std::string &topic, Finished finished = Finished());

  /**
   * \brief Subscribe the given default callback.
   *
   * The default callback is stored under the empty name \c "".
   */
  void subscribe(Callback callback);

  /**
   * \brief Subscribe the given named callback.
   *
   * A named callback can later be unsubscribed by its name.
   */
  void subscribe(const std::string& name, Callback callback);

  /**
   * \brief Subscribe the given named callback.
   *
   * A named callback can later be unsubscribed by its name.
   */
  void subscribe(const std::string& name, uint32_t queue_size, Callback callback);
  
  /**
   * \brief Subscribe the given object callback as default.
   */
  template<class T>
  void subscribe(void(T::*callback)(Video&, Frame&), T* object);
  
  /**
   * \brief Subscribe the given object callback under the given name.
   */
  template<class T>
  void subscribe(const std::string& name, void(T::*callback)(Video&, Frame&), T* object);

  /**
   * \brief Pause the given (playing) video.
   *
   * If there is no replaying video at the given path, or the given name does not
   * refer to a replaying video, this method has no effect.
   */
  void pause(const std::string &path);

  /**
   * \brief Resume the given (paused) video.
   *
   * If there is no replaying video at the given path, or the given name does not
   * refer to a paused video, this method has no effect.
   */
  void resume(const std::string &path);

  /**
   * \brief Remove a named operator: publisher, subscriber, recorder or replayer.
   */
  void stop(const std::string& name = "");
};

template<class T>
void Video::publish(const std::string& topic, const T& image, bool latch)
{
  if (operators_.count(topic) == 0)
    operators_[topic].reset(new Publisher(transport_, topic, latch));

  Publisher &publisher = dynamic_cast<Publisher&>(*operators_[topic]);

  publisher.publish(image);
}

template<class T>
void Video::subscribe(void(T::*callback)(Video&, Frame&), T* object)
{
  boost::function<void(T*, Video&, Frame&)> member_function = callback;
  subscribe(boost::bind(member_function, object, _1, _2));
}

template<class T>
void Video::subscribe(const std::string& name, void(T::*callback)(Video&, Frame&), T* object)
{
  boost::function<void(T*, Video&, Frame&)> member_function = callback;
  subscribe(name, boost::bind(member_function, object, _1, _2));
}

} //namespace cv_video

#endif
