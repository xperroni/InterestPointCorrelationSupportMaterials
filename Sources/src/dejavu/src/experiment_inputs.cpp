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

#include <dejavu/difference_stream.h>
#include <dejavu/file_stream.h>
#include <dejavu/settings.h>

#include <clarus/clarus.hpp>
using clarus::List;

#include <opencv2/opencv.hpp>

#include <ros/console.h>

#include <string>

namespace dejavu
{

static const int KEY_ESC = 27;

cv::Mat bgr(const cv::Mat &data)
{
  cv::Mat normalized = data - clarus::min(data);
  normalized /= clarus::max(normalized);
  return depths::bgr(fourier::swap(normalized));
}

cv::Mat differences(const cv::Mat &a, const cv::Mat &b)
{
  int channels = std::min(a.channels(), b.channels());
  if (channels > 1)
  {
    List<cv::Mat> u = clarus::split(a);
    List<cv::Mat> v = clarus::split(b);
    cv::Mat summed(a.size(), CV_64F, cv::Scalar(0));
    for (int i = 0; i < channels; i++)
      summed += differences(u[i], v[i]);

    return summed;
  }

  cv::Mat Ia = images::convert(a, CV_64F);
  cv::Mat Ib = images::convert(b, CV_64F);

  cv::Mat Fa = fourier::transform(Ia, cv::DFT_COMPLEX_OUTPUT);
  cv::Mat Fb = fourier::transform(Ib, cv::DFT_COMPLEX_OUTPUT);

  cv::Mat Aa = fourier::magnitude(Fa);
  cv::Mat Ab = fourier::magnitude(Fb);

  cv::Mat Pa = fourier::phase(Fa);
  cv::Mat Pb = fourier::phase(Fb);

  cv::Mat Ad = clarus::log(images::absdiff(Aa, Ab));
  cv::Mat Pd = images::absdiff(Pa, Pb);

  viewer::show("Amplitude", bgr(Ad));
  viewer::show("Phase", bgr(Pd));
  viewer::show("Pa", bgr(Pb));

  //cv::Mat A0(Ad.size(), Ad.type(), cv::Scalar(0));
  cv::Mat Ed = clarus::exp(Ad, Pb);
  cv::Mat Sd = clarus::pow(fourier::inverse(Ed), 2.0);

  return clarus::split(Sd)[0];
}

/*
cv::Mat differences(const cv::Mat &a, const cv::Mat &b)
{
  int channels = std::min(a.channels(), b.channels());
  if (channels > 1)
  {
    List<cv::Mat> u = clarus::split(a);
    List<cv::Mat> v = clarus::split(b);
    cv::Mat summed(a.size(), CV_64F, cv::Scalar(0));
    for (int i = 0; i < channels; i++)
      summed += differences(u[i], v[i]);

    return summed;
  }

  cv::Mat Ia = images::convert(a, CV_64F);
  cv::Mat Ib = images::convert(b, CV_64F);

  List<cv::Mat> Fa = clarus::split(fourier::transform(Ia, cv::DFT_COMPLEX_OUTPUT));
  List<cv::Mat> Fb = clarus::split(fourier::transform(Ib, cv::DFT_COMPLEX_OUTPUT));

  cv::Mat Ad = images::absdiff(Fa[0], Fb[0]);
  cv::Mat Ed = clarus::merge((List<cv::Mat>(), Ad, Fb[1]));
  cv::Mat Sd = clarus::pow(fourier::inverse(Ed), 2.0);

  return clarus::split(Sd)[0];
}
*/

cv::Mat get(FileStream &stream)
{
  cv::Mat image = stream();
  if (image.empty())
    return cv::Mat();

  return images::scale(image, 128);
}

void experiment_inputs()
{
  FileStream stream(0, param::path_teach());
  cv::Mat previous = get(stream);
  for (;;)
  {
    cv::Mat image = get(stream);
    if (image.empty())
      break;

    cv::Mat d = images::scale(differences(previous, image), 640);
    d -= clarus::min(d);
    d /= clarus::max(d);

    viewer::show("Differences", depths::bgr(d));

    cv::waitKey();

    previous = image;
  }
}

void experiment_inputs(int argc, char *argv[])
{
  ros::init(argc, argv, "experiment_inputs");

  experiment_inputs();

  while (ros::ok())
  {
    int key = cv::waitKey(20);
    if (key == KEY_ESC)
      return;

    ros::spinOnce();
  }
}

} // namespace dejavu

int main(int argc, char *argv[])
{
  dejavu::experiment_inputs(argc, argv);

  return 0;
}
