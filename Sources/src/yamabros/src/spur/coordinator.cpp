/*
This file is part of Yamabros.

Yamabros is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

Yamabros is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Yamabros. If not, see <http://www.gnu.org/licenses/>.
*/

#include <yamabros/yamabros.h>
#include <yamabros/settings.h>
#include <yamabros/spur/coordinator.h>

#include <ypspur.h>

#include <ros/ros.h>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>
namespace fs = boost::filesystem;

#include <cmath>
#include <iostream>
#include <stdexcept>

#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>

namespace yamabros
{

namespace spur
{

static size_t npos = std::string::npos;

// Create a named pipe for communication between ypspur-coordinator and the node process.
static std::string make_pipe()
{
  std::string path = (fs::temp_directory_path() / fs::unique_path()).native();
  int status = mkfifo(path.c_str(), 0666);
  if (status != 0) {
      throw std::runtime_error("Could not create named pipe \"" + path + '"');
  }

  return path;
}

// Run ypspur-coordinator as a background process.
static void run_ypspur_coordinator(const std::string &path)
{
  boost::format command_format("ypspur-coordinator -p %s -d %s");
  std::string command = (command_format % settings::spur::param_file() % settings::spur::device() ).str();
  ROS_INFO_STREAM(command);

  boost::format background_format("(%s > %s 2>&1) &");
  std::string background = (background_format % command % path).str();
  int status = system(background.c_str());
}

static FILE *open_pipe()
{
  std::string path = make_pipe();
  run_ypspur_coordinator(path);

  // Open the pipe on this side to receive output from ypspur-coordinator.
  return fopen(path.c_str(), "r");
}

Coordinator::Coordinator():
  pipe_(open_pipe())
{
  // Read output from ypspur-coordinator until a clear success or error message is received.
  for (std::string line = ""; line.find("Trajectory control loop started") == npos;)
  {
    if (!read(line))
    {
      std::string msg = "Error reading from ypspur-coordinator";
      ROS_ERROR_STREAM(msg);
      throw std::runtime_error(msg);
    }

    if (line.find("Error") != npos || line.find("error") != npos)
    {
      ROS_ERROR_STREAM(line);
      throw std::runtime_error(line);
    }

    ROS_INFO_STREAM(line);
  }
}

Coordinator::~Coordinator()
{
  fclose(pipe_);

  // Closing the pipe should be enough to stop the ypspur-coordinator process,
  // but this makes sure there are no instances left running.
  system("killall ypspur-coordinator > /dev/null 2>&1");
}

bool Coordinator::read(std::string &line)
{
  char *buffer = NULL;
  size_t size = 0;
  int count = getline(&buffer, &size, pipe_);
  if (count <= 0)
    return false;

  line = std::string(buffer);
  line.erase(line.find_last_not_of(" \n\r\t") + 1);
  free(buffer);
  return true;
}

} // namespace spur

} // namespace yamabros
