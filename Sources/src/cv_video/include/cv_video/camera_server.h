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

#ifndef CV_VIDEO_CAMERA_SERVER_H
#define CV_VIDEO_CAMERA_SERVER_H

#include <cv_video/video.h>
#include <cv_video/CommandAction.h>
#include <cv_video/SnapshotAction.h>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <boost/function.hpp>

#include <vector>

namespace cv_video
{

class CameraServer
{
  /** \brief Action implementor type. */
  typedef boost::function<void(const Record&)> Action;

  /** \brief ROS node handler. */
  ros::NodeHandle node_;

  /** \brief Action server. */
  actionlib::SimpleActionServer<CommandAction> recorder_server_;

  /** \brief Snapshot server. */
  actionlib::SimpleActionServer<SnapshotAction> snapshot_server_;

  /** \brief Video signal manager. */
  Video video_;

  /** \brief Path to the currently recording video file. */
  std::string path_;

  /** \brief List of implemented actions. */
  std::vector<Action> actions_;

  /**
   * \brief Accept a new action request.
   */
  void accept();

  /**
   * \brief Provide feedback on an ongoing action.
   */
  void feedback(Video& video, Frame& frame);

  /**
   * \brief Preempt an ongoing action.
   */
  void preempt();

  /**
   * \brief Open the camera in viewing mode.
   */
  void open(const Record& request);

  /**
   * \brief Open the camera in recording mode.
   */
  void record(const Record& request);

  /**
   * \brief Stop any ongoing camera operation.
   */
  void stop(const Record& request);

  /**
   * \brief Retrieves a snapshot from the camera.
   */
  void snapshot();

  /**
   * \brief Snapshot retrieval callback.
   */
  void snapshotResult(Video& video, Frame& frame);

public:
  /**
   * \brief Default constructor.
   */
  CameraServer(const std::string &name = "", const std::string &topic = "");

  /**
   * \brief Convenience alias for <tt>ros::spin()</tt>.
   */
  void spin();

  /**
   * \brief Convenience alias for <tt>ros::spinOnce()</tt>.
   */
  void spinOnce();

  /**
   * \brief Return a reference to the enclosed Video object.
   */
  Video &video();
};

} // namespace cv_video

#endif
