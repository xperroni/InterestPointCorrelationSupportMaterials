/*
 * Copyright (c) Helio Perroni Filho <xperroni@gmail.com>
 * 
 * This file is part of Dejavu.
 * 
 * Dejavu is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * Dejavu is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with Dejavu. If not, see <http://www.gnu.org/licenses/>.
 */


#include <dejavu/dispatcher.h>

#include <yamabros/spur.h>
using yamabros::spur::Spur;
using yamabros::spur::LEFT;
using yamabros::spur::RIGHT;

#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>

namespace dejavu
{

// TODO: implement a manual control option on the steering node.
#if 0
void test_line_input()
{
  terminal_io::Terminal terminal;
  for (;;)
  {
    terminal << "[1] Yaw" << std::endl;
    terminal << "[2] Sideways" << std::endl;
    terminal << "[3] Quit" << std::endl;
    terminal << "Select option: " << std::flush;

    int option = -1;
    terminal >> option;

    terminal << std::endl;

    ROS_INFO_STREAM(option);

    if (option == 3)
      break;
  }
}

static void event(terminal_io::Terminal &terminal, char key)
{
  int code = key;
  ROS_INFO_STREAM(code);
}

void test_key_input()
{
  terminal_io::Terminal terminal;
  terminal.get("Enter keystrokes: ", event, '\n');
  terminal << std::endl << "Done." << std::endl;
}

static void turn(Spur &spur, terminal_io::Terminal &terminal, char key)
{
  static double TURN = 1.0 * yamabros::DEGS; // M_PI / 18.0; // 10.0 * yamabros::DEGS

  if (key == 'a')
    spur.turn(yamabros::spur::LEFT);
  else if (key == 'd')
    spur.turn(yamabros::spur::RIGHT);
  else if (key == 's')
    spur.straight();
}

void teach()
{
  terminal_io::Terminal terminal;
  terminal.prompt("Press <ENTER> to start operation.");

  cv_video::Camera camera;
  camera.record();

  Spur spur(false);
  spur.straight();

  terminal.get("Press 'a' or 'd' to turn left and right. Press <ENTER> to stop.\n",
               boost::bind(turn, boost::ref(spur), _1, _2),
               '\n');

  camera.stop();
  spur.coast();
}
#endif

struct Steering: Dispatcher
{
  enum Direction {LEFTWARD, RIGHTWARD, STRAIGHT};

  Spur spur;

  bool moving;

  Direction direction;

  Steering():
    spur(false),
    moving(true),
    direction(STRAIGHT)
  {
    subscribe<std_msgs::Int32>(name("direction"), &Steering::updateDirection, this);
    subscribe<std_msgs::Bool>(name("forward"), &Steering::updateMoving, this);
    spur.straight();
  }

  void updateDirection(const std_msgs::Int32ConstPtr &message)
  {
    if (!moving)
      return;

    int value = message->data;
    Direction correction = (
      value < 0 ? LEFTWARD  :
      value > 0 ? RIGHTWARD :
      STRAIGHT
    );

    if (direction == correction)
      return;

    if (correction == STRAIGHT)
      spur.straight();
    else if (direction == STRAIGHT && correction == LEFTWARD)
      spur.turn(0.03);
    else if (direction == STRAIGHT && correction == RIGHTWARD)
      spur.turn(-0.03);
    else if (direction == RIGHTWARD && correction == LEFTWARD)
      spur.turn(0.06);
    else if (direction == LEFTWARD && correction == RIGHTWARD)
      spur.turn(-0.06);

    direction = correction;
  }

  void updateMoving(const std_msgs::BoolConstPtr &message)
  {
    if (moving == message->data)
      return;

    moving = message->data;
    if (moving)
      spur.straight();
    else
      spur.brake();
  }
};

} // namespace dejavu

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "steering");

  dejavu::Steering steer;

  ros::spin();
}
