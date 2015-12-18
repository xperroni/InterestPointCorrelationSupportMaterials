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

#ifndef CV_VIDEO_REPLAYER_H
#define CV_VIDEO_REPLAYER_H

#include <cv_video/publisher.h>

#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>

#include <opencv2/opencv.hpp>

namespace cv_video
{

class Replayer: public Operator
{
  /** \brief Callback type for signaling the video has finished playing. */
  typedef boost::function<void()> Callback;

  /** \brief Video replay object. */
  boost::shared_ptr<cv::VideoCapture> video_;

  /** \brief Publisher object used to publish replay frames. */
  Publisher::Ptr publisher_;

  /** \brief ROS timer used to issue video frames. */
  ros::Timer timer_;

  /** \brief Replay finish callback; */
  Callback callback_;

  /** \brief Flag indicating whether the video finished playing. */
  bool done_;

  /**
   * \brief Initialize this replayer.
   */
  void start(const std::string &path, Publisher::Ptr publisher, double fps, Callback callback);

  /**
   * \brief Initialize this replayer.
   */
  void start(const std::string &path, Callback callback);

  /**
   * \brief Callback for timer events.
   */
  void timerCallback(const ros::TimerEvent& event);

public:
  /**
   * \brief Create a new replay session for the video at the given path.
   *
   * Video frames are replayed to the given output topic. If the frame rate is omitted,
   * frames are replayed at the default frame rate specified by the \c ~fps parameter.
   *
   * \arg path Path to file to be replayed.
   * \arg publisher Pointer to object used to publish frames.
   * \arg fps Replay frame rate, in Frames Per Second (FPS). If omitted the \c ~fps parameter is used.
   * \arg callback Function to be called when the video finishes playing.
   */
  Replayer(const std::string &path, Publisher::Ptr publisher, double fps = 0, Callback callback = Callback());

  /**
   * \brief Create a new replay session for the video at the given path.
   *
   * Video frames are replayed to the given output topic at the frame rate
   * specified by the \c ~fps parameter.
   *
   * \arg path Path to file to be replayed.
   * \arg publisher Pointer to object used to publish frames.
   * \arg callback Function to be called when the video finishes playing.
   */
  Replayer(const std::string &path, Publisher::Ptr publisher, Callback callback = Callback());

  /**
   * \brief Create a new replay session for the video at the given path.
   *
   * Video frames are not replayed automatically; they can be retrieved by calls
   * to the <tt>next()</tt> method or the function operator.
   */
  Replayer(const std::string &path, Callback callback = Callback());

  /**
   * \brief Object destructor.
   *
   * Enforces polymorphism. Do not remove.
   */
  virtual ~Replayer();

  /**
   * \brief Function form of the <tt>next()</tt> method.
   */
  cv::Mat operator () ();

  /**
   * \brief Returns whether all video frames have already been played.
   */
  bool done();

  /**
   * \brief Returns the next video frame.
   *
   * If this replay session is active (i.e. it was created with any of the constructors
   * that take a ROS node handler as argument) the frame returned is the last replayed one.
   * Otherwise a new frame is grabbed from the video file and returned.
   *
   * If no frames remain to be replayed, return an empty image.
   */
  cv::Mat next();

  /**
   * \brief Pauses this replay session.
   *
   * Replay can be resumed by calling the <tt>resume()</tt> method.
   */
  void pause();

  /**
   * \brief Resume a paused replay session.
   *
   * Has no effect if the replay is not paused.
   */
  void resume();

  /**
   * \brief Stops this replay session.
   */
  void stop();
};

} // namespace cv_video

#endif
