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

#ifndef CV_VIDEO_CAMERA_H
#define CV_VIDEO_CAMERA_H

#include <cv_video/frame.h>
#include <cv_video/CommandAction.h>
#include <cv_video/SnapshotAction.h>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>

namespace cv_video
{

class Camera
{
  typedef actionlib::SimpleActionClient<CommandAction> ActionClient;

  typedef actionlib::SimpleActionClient<SnapshotAction> SnapshotClient;

  enum Mode
  {
    OPEN,
    RECORD,
    STOP
  };

  /** \brief Reference to the current ROS node. */
  ros::NodeHandle node_;

  /** \brief Action client. */
  boost::shared_ptr<ActionClient> action_client_;

  /** \brief Action client for retrieving individual camera snapshots. */
  boost::shared_ptr<SnapshotClient> snapshot_client_;
  
  /**
   * \brief Send an action request to the server.
   */
  void send(Mode mode, const Record& request, ActionClient::SimpleFeedbackCallback feedback);

public:
  /** \brief Callback type to fetch frames as they are recorded */
  typedef boost::function<void(Camera&, Frame&)> Callback;

  /** \brief Default constructor. */
  Camera(bool spin_thread = true);

  /** \brief Creates a new camera connected to the given server. */
  Camera(const std::string &name, bool spin_thread = true);

  /** \brief Object destructor. */
  virtual ~Camera();

  /**
   * \brief Return a camera snapshot as an OpenCV image.
   */
  cv::Mat operator() ();

  /**
   * \brief Starts a new recording.
   * 
   * Recording parameters are taken from environment parameters, or if these are unavailable,
   * set to reasonable defaults.
   */
  void record(Callback callback = Callback());

  /**
   * \brief Starts a new recording to the given output file.
   *
   * Recording parameters are taken from environment parameters, or if these are unavailable,
   * set to reasonable defaults.
   */
  void record(const std::string& path, Callback callback = Callback());

  /** \brief Starts a new recording. */
  void record(const std::string& path,
              const std::string& format,
              double fps,
              int width,
              int height,
              Callback callback = Callback());

  /** \brief Stops an ongoing recording. */
  void stop();

  /**
   * \brief Return a frame from the connected camera.
   */
  Frame snapshot();
};

} // namespace cv_video

#endif
