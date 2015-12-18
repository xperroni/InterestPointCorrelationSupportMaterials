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

double cos(const cv::Mat &a, const cv::Mat &b)
{
  double dot = 0.0;
  double a_m = 0.0;
  double b_m = 0.0;

  int rows = a.rows;
  int cols = a.cols;
  for (int i = 0; i < rows; i++)
  {
    for (int j = 0; j < cols; j++)
    {
      double a_k = a.at<double>(i, j);
      double b_k = b.at<double>(i, j);
      dot += a_k * b_k;
      a_m += a_k * a_k;
      b_m += b_k * b_k;
    }
  }

  return dot / (sqrt(a_m) * sqrt(b_m));
}

cv::Mat bgr(const cv::Mat &data)
{
  cv::Mat normalized = data - clarus::min(data);
  normalized /= clarus::max(normalized);
  return depths::bgr(fourier::swap(normalized));
}

List<cv::Mat> differences(const std::string &name, const cv::Mat &a, const cv::Mat &b)
{
  int channels = std::min(a.channels(), b.channels());
  if (channels > 1)
  {
    List<cv::Mat> u = clarus::split(a);
    List<cv::Mat> v = clarus::split(b);
    cv::Mat As;
    cv::Mat Ps;
    for (int i = 0; i < channels; i++)
    {
      List<cv::Mat> d = differences(name, u[i], v[i]);
      if (i == 0)
      {
        As = d[0];
        Ps = d[1];
        continue;
      }

      As += d[0];
      Ps += d[1];
    }

    return (List<cv::Mat>(), As, Ps);
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

  viewer::show(name + " Amplitude", bgr(Ad));
  viewer::show(name + " Phase", bgr(Pd));

  return (List<cv::Mat>(), Ad, Pd);
}

cv::Mat get(FileStream &stream)
{
  cv::Mat image = stream();
  if (image.empty())
    return cv::Mat();

  return images::scale(image, 128);
}

struct Session
{
  FileStream stream;

  int index;

  std::string name;

  cv::Mat previous;

  Session(const std::string &name, const std::string &path):
    stream(0, path),
    index(1)
  {
    this->name = name;
    previous = get(stream);
  }

  List<cv::Mat> operator() ()
  {
    cv::Mat image = get(stream);
    if (image.empty())
      return List<cv::Mat>();

    List<cv::Mat> data = differences(name, previous, image);
    previous = image;
    return data;
  }
};

void experiment_spectra()
{
  Session teach("teach", param::path_teach());
  Session replay("replay", param::path_replay());
/*
  for (int i = 0; i < 100; i++)
    teach();
*/
  for (;;)
  {
    List<cv::Mat> t = teach();
    List<cv::Mat> r = replay();
    if (t.empty() || r.empty())
      return;

    cv::Mat Ad = images::absdiff(t[0], r[0]);
    cv::Mat Pd = images::absdiff(t[1], r[1]);

    ROS_INFO_STREAM("sum = " << cv::sum(Pd)[0]);

    viewer::show("Ad", bgr(Ad));
    viewer::show("Pd", bgr(Pd));

    cv::waitKey();
  }
}

void experiment_spectra(int argc, char *argv[])
{
  ros::init(argc, argv, "experiment_spectra");

  experiment_spectra();

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
  dejavu::experiment_spectra(argc, argv);

  return 0;
}
