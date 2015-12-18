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

#include <terminal_io/actions.h>
#include <terminal_io/settings.h>

#include <boost/bind.hpp>

#include <iostream>

#include <termios.h>

namespace terminal_io
{

Actions::Actions():
  action_server_(node_, settings::name(), boost::bind(&Actions::callback, this, _1), false)
{
  actions_.push_back(boost::bind(&Actions::getKey, this, _1));
  actions_.push_back(boost::bind(&Actions::getLine, this, _1));
  actions_.push_back(boost::bind(&Actions::print, this, _1));

  // Black magic to prevent Linux from buffering keystrokes.
  // TODO: make multi-platform
  struct termios t;
  tcgetattr(STDIN_FILENO, &t);
  t.c_lflag &= ~ICANON;
  tcsetattr(STDIN_FILENO, TCSANOW, &t);

  action_server_.start();
}

void Actions::callback(const ExchangeGoalConstPtr &goal)
{
  Action action = actions_[goal->mode];
  action(goal->text);
}

void Actions::getKey(const std::string &text)
{
  char escape = text[0];

  if (text.size() > 1)
    std::cout << text.substr(1) << std::flush;

  for (ExchangeFeedback feedback;;)
  {
    if (action_server_.isPreemptRequested() || !ros::ok())
    {
      action_server_.setPreempted();
      return;
    }

    char key = std::cin.get();
    if (key == escape)
      break;

    feedback.key.assign(1, key);
    action_server_.publishFeedback(feedback);
  }

  ExchangeResult result;
  result.line = "";
  action_server_.setSucceeded(result);
}

void Actions::getLine(const std::string &text)
{
  if (text.size() > 0)
    std::cout << text << std::flush;

  ExchangeResult result;
  std::getline(std::cin, result.line);
  action_server_.setSucceeded(result);
}

void Actions::print(const std::string &text)
{
  std::cout << text << std::flush;

  ExchangeResult result;
  result.line = "";
  action_server_.setSucceeded(result);
}

} // namespace terminal_io
