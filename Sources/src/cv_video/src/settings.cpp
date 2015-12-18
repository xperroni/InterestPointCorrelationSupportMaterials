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

#include <cv_video/settings.h>

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.h>
namespace enc = sensor_msgs::image_encodings;

#include <boost/filesystem.hpp>
using boost::filesystem::current_path;

namespace cv_video
{

std::string encoding(const cv::Mat& image)
{
  switch (image.type())
  {
    case CV_8UC1:
    {
      return enc::MONO8;
    }
    case CV_8UC3:
    {
      return enc::BGR8;
    }
    default:
    {
      throw cv_bridge::Exception("Incompatible encoding (not CV_8UC1 nor CV_8UC3)");
    }
  }
}

Record params()
{
  Record record;
  record.path = param::path();
  record.format = param::format();
  record.fps = param::fps();
  record.width = param::width();
  record.height = param::height();

  return record;
}

namespace name
{

std::string image()
{
  return ros::names::resolve("image");
}

std::string camcorder()
{
  return ros::names::resolve("camcorder");
}

std::string playing()
{
  return ros::names::resolve("playing");
}

std::string snapshot()
{
  return ros::names::resolve("camcorder_snapshot");
}

} // namespace name

namespace param
{

std::string encoding()
{
  return param<std::string>("~encoding", "");
}

std::string format()
{
  return param<std::string>("~format", "MPEG");
}

double fps()
{
  return param<double>("~fps", 30.0);
}

std::string path()
{
  return param<std::string>("~path", (current_path() / "video.mpg").native());
}

bool playing()
{
  return param<bool>("~playing", true);
}

int width()
{
  return param<int>("~width", 640);
}

int height()
{
  return param<int>("~height", 480);
}

} // namespace param

} // namespace cv_video
