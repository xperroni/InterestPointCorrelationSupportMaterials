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

#ifndef CV_VIDEO_SETTINGS_H
#define CV_VIDEO_SETTINGS_H

#include <cv_video/Record.h>

#include <ros/ros.h>

#include <opencv2/opencv.hpp>

#include <string>

namespace cv_video
{

/**
 * \brief Select the appropriate cv_bridge encoding for the given image.
 */
std::string encoding(const cv::Mat& image);

/**
 * \brief Create a recording request.
 * 
 * Request values are taken from environment parameters, or sensible defaults
 * if unavailable.
 */
Record params();

namespace name
{

std::string image();

std::string camcorder();

std::string playing();

std::string snapshot();

} // namespace name

namespace param
{

template<class T> T param(const std::string& name, const T& fallback)
{
  T value;
  return (ros::param::get(name, value) ? value : fallback);
}

std::string encoding();

std::string format();

double fps();

std::string path();

bool playing();

int width();

int height();

} // namespace param

} // namespace cv_video

#endif
