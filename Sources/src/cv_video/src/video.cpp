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

#include <cv_video/video.h>
#include <cv_video/recorder.h>
#include <cv_video/replayer.h>
#include <cv_video/subscriber.h>
#include <cv_video/settings.h>

namespace cv_video
{

Video::Video():
    transport_(node_),
    topic_(name::image())
{
  // Nothing to do.
}

Video::Video(const std::string& topic):
    transport_(node_),
    topic_(topic)
{
  // Nothing to do.
}

void Video::record()
{
  record(param::path());
}

void Video::record(const std::string& path)
{
  record(path, param::format(), param::fps(), param::width(), param::height());
}

void Video::record(const Record& request)
{
  record(request.path,
         request.format,
         request.fps,
         request.width,
         request.height);
}

void Video::record(const std::string& path,
                   const std::string& format,
                   double fps,
                   int width,
                   int height)
{
  Recorder recorder(path, format, fps, width, height);
  subscribe(path, recorder);
}

void Video::replay(const std::string &path, const std::string &topic, double fps)
{
  Publisher::Ptr publisher(new Publisher(transport_, topic));
  operators_[path].reset(new Replayer(path, publisher, fps));
}

void Video::replay(const std::string &path, const std::string &topic, Finished finished)
{
  Publisher::Ptr publisher(new Publisher(transport_, topic));
  operators_[path].reset(new Replayer(path, publisher, finished));
}

void Video::subscribe(Callback callback)
{
  subscribe("", callback);
}

void Video::subscribe(const std::string& name, Callback callback)
{
  operators_[name].reset(new Subscriber(transport_, topic_, 1, callback, this));
}

void Video::subscribe(const std::string& name, uint32_t queue_size, Callback callback)
{
  operators_[name].reset(new Subscriber(transport_, topic_, queue_size, callback, this));
}

void Video::pause(const std::string &path)
{
  if (operators_.count(path) == 0)
    return;

  Replayer *replayer = dynamic_cast<Replayer*>(operators_[path].get());
  if (replayer == NULL)
    return;

  replayer->pause();
}

void Video::resume(const std::string &path)
{
  if (operators_.count(path) == 0)
    return;

  Replayer *replayer = dynamic_cast<Replayer*>(operators_[path].get());
  if (replayer == NULL)
    return;

  replayer->resume();
}

void Video::stop(const std::string& name)
{
  operators_.erase(name);
}

} //namespace cv_video
