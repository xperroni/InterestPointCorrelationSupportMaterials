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

#include <cv_video/camera.h>
#include <cv_video/settings.h>

#include <boost/bind.hpp>

namespace cv_video
{

Camera::Camera(bool spin_thread):
  action_client_(new ActionClient(name::camcorder(), spin_thread)),
  snapshot_client_(new SnapshotClient(name::snapshot(), spin_thread))
{
  action_client_->waitForServer();
  snapshot_client_->waitForServer();
}

Camera::Camera(const std::string& name, bool spin_thread):
  action_client_(new ActionClient(name, spin_thread)),
  snapshot_client_(new SnapshotClient(name + "_snapshot", spin_thread))
{
  action_client_->waitForServer();
  snapshot_client_->waitForServer();
}

Camera::~Camera()
{
  stop();
}

cv::Mat Camera::operator() ()
{
  return snapshot().copy();
}

void Camera::send(Mode mode, const Record& request, ActionClient::SimpleFeedbackCallback feedback)
{
  static ActionClient::SimpleDoneCallback done = ActionClient::SimpleDoneCallback();
  static ActionClient::SimpleActiveCallback active = ActionClient::SimpleActiveCallback();

  CommandGoal goal;
  goal.mode = mode;
  goal.request = request;
  action_client_->sendGoal(goal, done, active, feedback);
}

Frame Camera::snapshot()
{
  SnapshotGoal goal;
  snapshot_client_->sendGoal(goal);
  snapshot_client_->waitForResult();

  SnapshotResultConstPtr result = snapshot_client_->getResult();
  return Frame(result->image);
}

void Camera::record(Callback callback)
{
  record(param::path(), callback);
}

void Camera::record(const std::string& path, Callback callback)
{
  record(path,
         param::format(),
         param::fps(),
         param::width(),
         param::height(),
         callback);
}

static void callbackHandler(Camera* camera, Camera::Callback callback, const sensor_msgs::Image& image)
{
  if (callback.empty())
    return;

  try
  {
    Frame frame(image);
    callback(*camera, frame);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
  catch (cv::Exception& e)
  {
    ROS_ERROR("OpenCV exception: %s", e.what());
  }
  catch (...)
  {
    ROS_ERROR("Unknown error");
  }
}

static void feedbackHandler(Camera* camera, Camera::Callback callback, const CommandFeedbackConstPtr& feedback)
{
  callbackHandler(camera, callback, feedback->frame);
}

void Camera::record(const std::string& path,
                    const std::string& format,
                    double fps,
                    int width,
                    int height,
                    Callback callback)
{
  Record request;
  request.path = path;
  request.format = format;
  request.fps = fps;
  request.width = width;
  request.height = height;

  send(RECORD, request, boost::bind(feedbackHandler, this, callback, _1));
}

void Camera::stop()
{
  static ActionClient::SimpleFeedbackCallback feedback = ActionClient::SimpleFeedbackCallback();

  send(STOP, Record(), feedback);
}

} // namespace cv_video
