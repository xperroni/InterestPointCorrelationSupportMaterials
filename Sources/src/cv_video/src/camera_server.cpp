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

#include <boost/bind.hpp>

namespace cv_video
{

static std::string name_recorder(ros::NodeHandle &node, const std::string &name)
{
  return (name == "" ? name::camcorder() : node.resolveName(name));
}

static std::string name_snapshot(ros::NodeHandle &node, const std::string &name)
{
  return (name == "" ? name::snapshot() : node.resolveName(name) + "_snapshot");
}

static std::string topic_video(const std::string &topic)
{
  return (topic == "" ? name::image() : topic);
}

CameraServer::CameraServer(const std::string &name, const std::string &topic):
  node_(),
  recorder_server_(node_, name_recorder(node_, name), false),
  snapshot_server_(node_, name_snapshot(node_, name), false),
  video_(topic_video(topic)),
  path_("")
{
  actions_.push_back(boost::bind(&CameraServer::open, this, _1));
  actions_.push_back(boost::bind(&CameraServer::record, this, _1));
  actions_.push_back(boost::bind(&CameraServer::stop, this, _1));

  recorder_server_.registerGoalCallback(boost::bind(&CameraServer::accept, this));
  recorder_server_.registerPreemptCallback(boost::bind(&CameraServer::preempt, this));
  snapshot_server_.registerPreemptCallback(boost::bind(&CameraServer::snapshot, this));

  recorder_server_.start();
}

void CameraServer::accept()
{
  CommandGoalConstPtr goal = recorder_server_.acceptNewGoal();
  if (recorder_server_.isPreemptRequested())
    return;

  Action action = actions_[goal->mode];
  action(goal->request);
}

void CameraServer::feedback(Video& video, Frame& frame)
{
  if (!recorder_server_.isActive())
    return;

  sensor_msgs::ImageConstPtr image = frame.buffer();
  CommandFeedback feedback;
  feedback.frame = *image;

  recorder_server_.publishFeedback(feedback);
}

void CameraServer::preempt()
{
  video_.stop("feedback");

  recorder_server_.setPreempted();

  video_.stop();
  if (path_ != "")
  {
    video_.stop(path_);
    path_ = "";
  }

  std::string command = (path_ != "" ? "record" : "open");
  ROS_INFO_STREAM(command << "() preempted");
}

void CameraServer::open(const Record& request)
{
  video_.subscribe("feedback", &CameraServer::feedback, this);

  ROS_INFO_STREAM("open() started");
}

void CameraServer::record(const Record& request)
{
  path_ = request.path;
  video_.record(request);
  video_.subscribe("feedback", &CameraServer::feedback, this);

  ROS_INFO_STREAM("record() started");
}

void CameraServer::stop(const Record& request)
{
  video_.stop("feedback");

  CommandResult result;
  result.status = 0;
  recorder_server_.setSucceeded(result);
}

void CameraServer::snapshot()
{
  if (snapshot_server_.isPreemptRequested())
    return;

  video_.subscribe("snapshot", &CameraServer::snapshotResult, this);
}

void CameraServer::snapshotResult(Video& video, Frame& frame)
{
  video_.stop("snapshot");

  SnapshotResult result;
  sensor_msgs::ImageConstPtr image = frame.buffer();
  result.image = *image;

  snapshot_server_.setSucceeded(result);
}

void CameraServer::spin()
{
  ros::spin();
}

void CameraServer::spinOnce()
{
  ros::spinOnce();
}

Video &CameraServer::video()
{
  return video_;
}

} // namespace cv_video
