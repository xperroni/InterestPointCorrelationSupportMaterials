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

#include <terminal_io/terminal.h>
#include <terminal_io/settings.h>

namespace terminal_io
{

Terminal::Terminal():
  action_client_(settings::name(), true)
{
  action_client_.waitForServer();
}

Terminal::Terminal(const std::string &name):
  action_client_(name, true)
{
  action_client_.waitForServer();
}

Terminal &Terminal::operator << (std::ostream& (*manipulator)(std::ostream&))
{
  static std::ostream& (*std_endl)(std::ostream&) = std::endl;
  static std::ostream& (*std_flush)(std::ostream&) = std::flush;

  buffer_out_ << manipulator;
  if (manipulator == std_flush || manipulator == std_endl)
    flush();

  return *this;
}

void Terminal::send(Mode mode, const std::string &text, FeedbackCallback feedback, bool block)
{
  static ActionClient::SimpleDoneCallback done = ActionClient::SimpleDoneCallback();
  static ActionClient::SimpleActiveCallback active = ActionClient::SimpleActiveCallback();

  ExchangeGoal goal;
  goal.mode = mode;
  goal.text = text;
  action_client_.sendGoal(goal, done, active, feedback);

  if (block)
    action_client_.waitForResult();
}

void Terminal::flush()
{
  send(PRINT, buffer_out_.str());
  buffer_out_.str("");
}

std::string Terminal::prompt(const std::string &text)
{
  send(LINE, text);
  return action_client_.getResult()->line;
}

void Terminal::get(KeyEvent event, char escape, bool block)
{
  get("", event, escape, block);
}

static void feedbackHandler(Terminal *terminal, Terminal::KeyEvent event, const ExchangeFeedbackConstPtr &feedback)
{
  event(*terminal, feedback->key[0]);
}

void Terminal::get(const std::string &text, KeyEvent event, char escape, bool block)
{
  FeedbackCallback feedback = boost::bind(feedbackHandler, this, event, _1);
  send(KEY, escape + text, feedback, block);
}

} // namespace terminal_io
