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
#include <dejavu/feature_selector.h>
#include <dejavu/settings.h>

#include <clarus/clarus.hpp>
using clarus::List;

#include <boost/function.hpp>

#include <ros/console.h>

#include <string>

namespace dejavu
{

static const int KEY_ESC = 27;

static cv::Scalar WHITE(255, 255, 255);

static void normalize_inline(cv::Mat &data)
{
  data -= clarus::min(data);
  double w = clarus::max(data);
  if (w != 0)
    data /= w;
}

static cv::Mat bgr(const cv::Mat &data)
{
  cv::Mat normed = data - clarus::min(data);
  double w = clarus::max(normed);
  if (w != 0)
    normed /= w;

  return depths::bgr(fourier::swap(normed));
}

static cv::Mat difference(const cv::Mat &a, const cv::Mat &b)
{
  int channels = std::min(a.channels(), b.channels());
  if (channels > 1)
  {
    List<cv::Mat> u = clarus::split(a);
    List<cv::Mat> v = clarus::split(b);
    cv::Mat summed(a.size(), CV_64F, cv::Scalar(0));
    for (int i = 0; i < channels; i++)
      summed += difference(u[i], v[i]);

    return summed;
  }

  cv::Mat I = images::convert(images::absdiff(a, b), CV_64F);
  cv::Mat F = fourier::transform(I, cv::DFT_COMPLEX_OUTPUT);
  cv::Mat Pf = fourier::phase(F);
  cv::Mat A0 = cv::Mat(Pf.size(), Pf.type(), cv::Scalar::all(0));
  cv::Mat Ef = clarus::exp(A0, Pf);
  cv::Mat Sf = clarus::pow(fourier::inverse(Ef), 2.0);

  return clarus::split(Sf)[0] * 255.0;
}

static cv::Mat get(boost::function<cv::Mat()> stream)
{
  cv::Mat image = stream();
  if (image.empty())
    return cv::Mat();

  return images::scale(image, 256);
}

struct DiVS
{
  struct Session
  {
    List<cv::Mat> differences;

    int padding;

    Session(const std::string &name, const std::string &path, int p):
      padding(p)
    {
      ROS_INFO_STREAM(name << " records bootstrap");

      FileStream stream(5, path);
      cv::Mat previous = get(stream);
      for (;;)
      {
        cv::Mat image = get(stream);
        if (image.empty())
          break;

        differences.append(difference(previous, image));
        previous = image;
      }

      ROS_INFO_STREAM(name << " records bootstrap done (" << differences.size() << " entries)");
    }
  };

  struct Teach: public Session
  {
    Teach():
      Session("Teach", param::path_teach(), param::teach_padding())
    {
      // Nothing to do.
    }
  };

  struct Replay: public Session
  {
    Replay():
      Session("Replay", param::path_replay(), param::replay_padding())
    {
      // Nothing to do.
    }
  };

  Teach teach;

  Replay replay;

  cv::Mat similarities;

  DiVS()
  {
    similarities = cv::Mat(teach.differences.size(), replay.differences.size(), CV_64F, cv::Scalar::all(0));
  }

  void relate(int j)
  {
    cv::Mat replayed = replay.differences[j];

    viewer::show("Replay", depths::bgr(replayed));

    cv::Mat column_similarities(similarities, cv::Rect(j, 0, 1, similarities.rows));
    for (int i = 0, n = teach.differences.size(); i < n; i++)
    {
      cv::Mat correlated = fourier::correlate(teach.differences[i], replayed, false);
      column_similarities.at<double>(i, 0) = clarus::max(correlated(cv::Rect(0, 0, correlated.cols, 1)));
      viewer::show("Responses", bgr(correlated));
    }

    normalize_inline(column_similarities);

    viewer::show("Similarities", depths::bgr(similarities));
  }
/*
  void relate(int j)
  {
    cv::Mat replayed = replay.differences[j];

    viewer::show("Replay", depths::bgr(replayed));

    cv::Mat column_similarities(similarities, cv::Rect(j, 0, 1, similarities.rows));

    List<FeaturePoint> features = selectSparseDifferences(30, replayed, replay.padding, teach.padding);
    cv::Mat responses_all(teach.differences.size(), features.size(), CV_64F, cv::Scalar(0));
    for (int k = 0, m = features.size(); k < m; k++)
    {
      cv::Mat responses = relate(features, k, teach.differences, 0, teach.differences.size());

      cv::Mat column_responses(responses_all, cv::Rect(k, 0, 1, responses_all.rows));
      responses.copyTo(column_responses);
      normalize_inline(column_responses);

      column_similarities += responses;
    }

    viewer::show("Responses", images::scale(depths::bgr(responses_all), cv::Size(600, teach.differences.size())));

    normalize_inline(column_similarities);

    viewer::show("Similarities", depths::bgr(similarities));

//    cv::waitKey();
  }
*/
  cv::Mat relate(const List<FeaturePoint> &features, int k, const List<cv::Mat> &images, int i0, int n)
  {
    const FeaturePoint &point = features[k];
    cv::Mat column(n, 1, CV_64F, cv::Scalar(0));
    for (int i = 0; i < n; i++)
    {
      const cv::Mat &image = images[i + i0];

      cv::Mat neighborhood(image, point.bounds_search);
      const cv::Mat &spectra_j = point.spectra;
      cv::Mat spectra_i = fourier::transform(fourier::normalize(neighborhood), 0, spectra_j.size());

      cv::Mat correlated = fourier::multiply(spectra_i, spectra_j, true);
      cv::Mat responses = fourier::inverse(correlated, point.reach);
      column.at<double>(i, 0) = clarus::max(responses);
    }

    return column;
  }

  void spin()
  {
    for (int j = 0, m = replay.differences.size(); j < m && ros::ok(); j++)
      relate(j);

    while (ros::ok())
    {
      int key = cv::waitKey(20);
      if (key == KEY_ESC)
        return;

      ros::spinOnce();
    }
  }
};

void experiment_differences(int argc, char *argv[])
{
  ros::init(argc, argv, "experiment_differences");

  DiVS divs;

  divs.spin();
}

} // namespace dejavu

int main(int argc, char *argv[])
{
  dejavu::experiment_differences(argc, argv);

  return 0;
}
