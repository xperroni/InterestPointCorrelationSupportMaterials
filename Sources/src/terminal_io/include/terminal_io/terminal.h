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

#ifndef TERMINAL_IO_TERMINAL_H
#define TERMINAL_IO_TERMINAL_H

#include <terminal_io/ExchangeAction.h>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <iostream>
#include <sstream>

namespace terminal_io
{

class Terminal
{
  typedef actionlib::SimpleActionClient<ExchangeAction> ActionClient;

  typedef ActionClient::SimpleFeedbackCallback FeedbackCallback;

  /** \brief Action invocation modes. */
  enum Mode
  {
    KEY,
    LINE,
    PRINT
  };

  /** \brief ROS node handler. */
  ros::NodeHandle node_;

  /** \brief Spur command action client. */
  ActionClient action_client_;

  /** \brief Input buffer. */
  std::istringstream buffer_in_;

  /** \brief Output buffer. */
  std::ostringstream buffer_out_;

  void send(Mode mode,
            const std::string &text = "",
            FeedbackCallback feedback = FeedbackCallback(),
            bool block = true);

public:
  /** \brief Key press event listener type. */
  typedef boost::function<void(Terminal&, char)> KeyEvent;

  /**
   * \brief Default constructor.
   */
  Terminal();

  /**
   * \brief Create a terminal client connected to the given server.
   */
  Terminal(const std::string &name);

  template<class T>
  Terminal &operator >> (T &value);

  template<class T>
  Terminal &operator << (const T &value);

  Terminal &operator << (std::ostream& (*manipulator)(std::ostream&));

  /**
   * \brief Flushes the output buffer.
   */
  void flush();

  /**
   * \brief Request one input line from the terminal server.
   */
  std::string prompt(const std::string &text = "");

  /**
   * \brief Listen o the terminal for individual keystrokes.
   */
  void get(KeyEvent event, char escape = 'q', bool block = true);

  /**
   * \brief Listen o the terminal for individual keystrokes.
   */
  void get(const std::string &text, KeyEvent event, char escape = 'q', bool block = true);

  template<class T>
  void print(const T &value);

  template<class T>
  void printFlush(const T &value);

  template<class T>
  void printLine(const T &value);
};

template<class T>
Terminal &Terminal::operator >> (T &value)
{
  std::string line = prompt();
  buffer_in_.str(line);
  buffer_in_.seekg(std::ios_base::beg);
  buffer_in_ >> value;
  return *this;
}

template<class T>
Terminal &Terminal::operator << (const T &value)
{
  buffer_out_ << value;
  return *this;
}

template<class T>
void Terminal::print(const T &value)
{
  buffer_out_ << value;
}

template<class T>
void Terminal::printFlush(const T &value)
{
  buffer_out_ << value;
  flush();
}

template<class T>
void Terminal::printLine(const T &value)
{
  buffer_out_ << value << std::endl;
  flush();
}

} // namespace terminal_io

#endif
