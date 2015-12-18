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

#include <clarus/clarus.hpp>
using clarus::List;

#include <cv_video/video.h>
using cv_video::Video;
using cv_video::Frame;

cv::Mat basis(const cv::Size &size, int k1, int k2)
{
  cv::Mat b(size, CV_64F, cv::Scalar::all(0.0));
  b.at<double>(k1, k2) = 255.0;

  /*cv::Mat b = cv::Mat(size, CV_64F);
  cv::randu(b, cv::Scalar::all(0), cv::Scalar::all(255));*/

  return fourier::transform(b, cv::DFT_COMPLEX_OUTPUT);
}

void showBasis(Video &video, const cv::Mat &b)
{
  List<cv::Mat> b_parts = clarus::split(b);
  cv::Mat b_r = colors::discrete(b_parts[0]);
  cv::Mat b_i = colors::discrete(b_parts[1]);

  video.publish("/hrr/basis/fourier/r", b_r);
  video.publish("/hrr/basis/fourier/i", b_i);

  cv::Mat s = fourier::inverse(b);
  List<cv::Mat> parts = clarus::split(s);
  cv::Mat r = colors::discrete(parts[0]);
  cv::Mat i = colors::discrete(parts[1]);

  video.publish("/hrr/basis/r", r);
  video.publish("/hrr/basis/i", i);
}

void showConvolution(Video &video, const cv::Mat &image, const cv::Mat &base)
{
  cv::Mat F = fourier::transform(image, cv::DFT_COMPLEX_OUTPUT);
  cv::Mat S = fourier::multiply(F, base);
  cv::Mat R = fourier::multiply(S, F, true);
  List<cv::Mat> C = clarus::split(fourier::inverse(R));
  video.publish("/hrr/image/r", colors::discrete(C[0]));
  video.publish("/hrr/image/i", colors::discrete(C[1]));
}

cv::Mat load64F(const std::string &path)
{
  return fourier::transform(images::convert(colors::grayscale(images::load(path)), CV_64F), cv::DFT_COMPLEX_OUTPUT);
}

cv::Mat bind(const cv::Mat &I, const cv::Mat &B)
{
  return fourier::multiply(I, B);
}

cv::Mat unbind(const cv::Mat &T, const cv::Mat &B)
{
  return fourier::multiply(T, B, true);
}

void show(Video &video, const std::string &root, const cv::Mat &F)
{
  List<cv::Mat> i = clarus::split(fourier::inverse(F));
  video.publish(root + "/spatial/r", colors::discrete(i[0]));
  //video.publish(root + "/spatial/i", colors::discrete(i[1]));

  //List<cv::Mat> C = clarus::split(F);
  //video.publish(root + "/fourier/r", colors::discrete(C[0]));
  //video.publish(root + "/fourier/i", colors::discrete(C[1]));
}

void show(Video &video, const std::string &root, const cv::Mat &F, int i, int j)
{
  List<cv::Mat> parts = clarus::split(fourier::inverse(F));
  cv::Mat values = parts[0];
  cv::Mat grays = colors::discrete(values);
  cv::Mat bgr = colors::convert(grays, CV_GRAY2BGR);

  cv::Vec3b &base = bgr.at<cv::Vec3b>(i, j);
  base[0] = 0;
  base[1] = 255;
  base[2] = 0;

  cv::Point p = clarus::argmax(values);
  cv::Vec3b &guess = bgr.at<cv::Vec3b>(p.y, p.x);
  guess[0] = 0;
  guess[1] = 0;
  guess[2] = 255;

  video.publish(root + "/spatial/r", bgr);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "saliences");

  cv::Mat i1 = load64F("/home/helio/Roboken/Projects/Active/Lividum/scene1.png");
  cv::Mat b1 = basis(i1.size(), 80, 80);
  cv::Mat t1 = bind(i1, b1);

  cv::Mat i2 = load64F("/home/helio/Roboken/Projects/Active/Lividum/scene2.png");
  cv::Mat b2 = basis(i1.size(), 330, 330);
  cv::Mat t2 = bind(i2, b2);

  cv::Mat i3 = load64F("/home/helio/Roboken/Projects/Active/Lividum/scene3.png");
  cv::Mat b3 = basis(i1.size(), 80, 330);
  cv::Mat t3 = bind(i3, b3);

  cv::Mat i4 = load64F("/home/helio/Roboken/Projects/Active/Lividum/scene4.png");

  cv::Mat t = t1 + t2 + t3;

  cv::Mat u1 = unbind(t, i1);
  cv::Mat u2 = unbind(t, i2);
  cv::Mat u3 = unbind(t, i3);

  // Attempt to recover b3 by unbinding an input similar to i3
  cv::Mat u4 = unbind(t, i4);

  Video video;
  for (ros::Rate rate(1); ros::ok();)
  {
    show(video, "/hrr/unbound1", u1, 80, 80);
    show(video, "/hrr/unbound2", u2, 330, 330);
    show(video, "/hrr/unbound3", u3, 80, 330);
    show(video, "/hrr/unbound4", u4, 80, 330);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}