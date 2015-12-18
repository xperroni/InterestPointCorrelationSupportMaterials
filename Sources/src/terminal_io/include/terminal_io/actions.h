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

#ifndef TERMINAL_IO_ACTIONS_H
#define TERMINAL_IO_ACTIONS_H

#include <terminal_io/ExchangeAction.h>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <boost/function.hpp>

#include <vector>

namespace terminal_io
{

class Actions
{
  /** \brief Action implementor type. */
  typedef boost::function<void(const std::string&)> Action;

  /** \brief List of implemented actions. */
  std::vector<Action> actions_;

  /** \brief ROS node handler. */
  ros::NodeHandle node_;

  /** \brief Action server. */
  actionlib::SimpleActionServer<ExchangeAction> action_server_;

  /**
   * \brief Action callback function.
   */
  void callback(const ExchangeGoalConstPtr &goal);

  /**
   * \brief Read individual key strokes from the terminal.
   */
  void getKey(const std::string &text);

  /**
   * \brief Read one line from the terminal.
   */
  void getLine(const std::string &text);

  /**
   * \brief Print text to the terminal.
   */
  void print(const std::string &text);

public:
  /**
   * \brief Default constructor.
   */
  Actions();
};

} // namespace terminal_io

#endif
