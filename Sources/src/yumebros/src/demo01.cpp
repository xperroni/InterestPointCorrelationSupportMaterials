/*
Copyright (c) Helio Perroni Filho <xperroni@gmail.com>

This file is part of Jamais.

Jamais is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

Jamais is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Jamais. If not, see <http://www.gnu.org/licenses/>.
*/

#include <yamabros/spur.h>

#include <cmath>

using yamabros::spur::Spur;

void demo01()
{
  Spur spur(true);
  spur.circle(3.0, 0.0, 1.0, 2.5 * M_PI);
  spur.circle(2.0, -0.15, 1.0, 2.5 * M_PI);
  spur.approach(1.15, 0.0, 0.0);
  spur.approach(1.5, 1.0, 0.25 * M_PI);
  spur.approach(2.12, 0.0, 0.0);
  spur.brake();
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "demo01");

  demo01();

  return 0;
}
