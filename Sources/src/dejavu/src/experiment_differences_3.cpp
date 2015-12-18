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
#include <dejavu/interpolator.h>
#include <dejavu/settings.h>

#include <clarus/clarus.hpp>
using clarus::List;

#include <boost/smart_ptr.hpp>
#include <boost/function.hpp>

#include <ros/console.h>

#include <iostream>
#include <fstream>
#include <string>

namespace dejavu
{

static std::string ROOT = "/home/helio/Roboken/Projects/Active/Lividum/output/";

static const int KEY_ESC = 27;

static cv::Scalar BLACK(0, 0, 0);

static cv::Scalar WHITE(255, 255, 255);

static double l_2PS = 1.0 / sqrt(2 * M_PI);

static int FEATURES = 2;

static int WINDOW_X = 20;

static int WINDOW_Y = 50;

static int TOP_POINTS = 5;

static void normalize_inline(cv::Mat &data)
{
  data -= clarus::min(data);
  double w = clarus::max(data);
  if (w != 0)
    data /= w;
}

static cv::Mat inverseMagnitudes(const cv::Mat &image, int padding)
{
  int side = 2 * padding + 1;
  cv::Rect area(0, 0, side, side);

  cv::Mat intensities = images::convert(image, CV_64F);
  cv::Mat intensities2 = intensities.mul(intensities);
  cv::Mat integral2 = images::integral(intensities2);

  int rows = integral2.rows - side;
  int cols = integral2.cols - side;
  cv::Mat inverted(rows, cols, CV_64F, cv::Scalar(side * side * 255.0));
  for (int i = 0; i < rows; i++)
  {
    area.y = i;
    for (int j = 0; j < cols; j++)
    {
      area.x = j;
      double m = sqrt(images::sum(integral2, area));
      if (m != 0)
        inverted.at<double>(i, j) = 1.0 / m;
    }
  }

  return inverted;
}

static cv::Mat spectralFilter(const cv::Mat &image)
{
  cv::Mat image2 = image.mul(image);
  double inv_m = 1.0 / sqrt(cv::sum(image2)[0]);
  return fourier::transform(image * inv_m);
}

static cv::Mat normalizedDiffs(const cv::Mat &image, const cv::Mat &inverse_magnitudes)
{
  int side = image.rows - inverse_magnitudes.rows;
  cv::Rect area(0, 0, side, side);

  int rows = inverse_magnitudes.rows;
  int cols = inverse_magnitudes.cols;
  cv::Mat summed(image.size(), CV_64F, cv::Scalar(0));
  for (int i = 0; i < rows; i++)
  {
    area.y = i;
    for (int j = 0; j < cols; j++)
    {
      area.x = j;
      cv::Mat patch(summed, area);
      patch += image(area) * inverse_magnitudes.at<double>(i, j);
    }
  }

  return summed;
}

static cv::Mat spectralFilter(const cv::Mat &image, const cv::Mat &inverse_magnitudes)
{
  int side = image.rows - inverse_magnitudes.rows;
  cv::Rect area(0, 0, side, side);

  int rows = inverse_magnitudes.rows;
  int cols = inverse_magnitudes.cols;
  cv::Mat summed(image.size(), CV_64F, cv::Scalar(0));
  for (int i = 0; i < rows; i++)
  {
    area.y = i;
    for (int j = 0; j < cols; j++)
    {
      area.x = j;
      cv::Mat patch(summed, area);
      patch += image(area) * inverse_magnitudes.at<double>(i, j);
    }
  }

  return fourier::transform(summed);
}

/*
static cv::Mat spectralMap(const cv::Mat &image, const cv::Mat &inverse_magnitudes, int padding, int padding2)
{
  cv::Mat summed(image.size(), CV_64F, cv::Scalar(0));
  List<FeaturePoint> features = selectSparseIntegrals(FEATURES, image, inverse_magnitudes, padding, padding2);
  for (int k = 0, n = features.size(); k < n; k++)
  {
    cv::Rect area = features[k].bounds_patch;
    int i = area.y;
    int j = area.x;

    cv::Mat patch(summed, area);
    patch += image(area) * inverse_magnitudes.at<double>(i, j);
  }

  return fourier::transform(summed);
}
*/

static cv::Mat spectralMap(const List<FeaturePoint> &features, const cv::Size &size, const cv::Mat &inverse_magnitudes)
{
  cv::Mat patch(features[0].patch.size(), CV_64F, cv::Scalar(0));
  for (int k = 0, n = features.size(); k < n; k++)
  {
    const FeaturePoint &point = features[k];
    cv::Rect area = point.bounds_patch;
    int i = area.y;
    int j = area.x;

    patch += point.patch; // * inverse_magnitudes.at<double>(i, j);
  }

  return fourier::transform(patch, 0, size);;
}

/*
static cv::Mat spectralMap(const cv::Mat &image, const cv::Mat &inverse_magnitudes, const List<FeaturePoint> &features)
{
  cv::Size size = image.size();
  cv::Mat summed(size, CV_64F, cv::Scalar(0));
  for (int k = 0, n = features.size(); k < n; k++)
  {
    cv::Rect area = features[k].bounds_patch;
    int i = area.y;
    int j = area.x;

    cv::Mat patch(summed, area);
    patch += image(area) * inverse_magnitudes.at<double>(i, j);
  }

  viewer::show("Teach", depths::bgr(summed));

  return fourier::transform(summed);
}
*/
static cv::Mat get(DifferenceStream &stream)
{
  cv::Mat image = stream();
  if (image.empty())
    return cv::Mat();

  return images::convert(images::scale(image, 256), CV_64F);
}

struct PointComp
{
  bool operator () (const cv::Point2d &a, const cv::Point2d &b)
  {
    return (a.y > b.y);
  }
};

static List<int> argmax(const cv::Mat &column, int n)
{
  std::set<cv::Point2d, PointComp> points;
  for (int i = 0, rows = column.rows; i < rows; i++)
    points.insert(cv::Point2d(i, column.at<double>(i, 0)));

  List<int> indexes;
  std::set<cv::Point2d, PointComp>::const_iterator i = points.begin(), m = points.end();
  for (int j = 0; i != m && j < n; ++i, ++j)
    indexes.append(i->x);

  return indexes;
}

struct DiVS
{
  struct Session
  {
    List<cv::Mat> differences;

    List<cv::Mat> spectra;

    List<cv::Mat> inverse_magnitudes;

    int padding;

    void load(const std::string &name, const std::string &path, int padding)
    {
      std::ofstream file_indexes((ROOT + name + "_indexes.txt").c_str());
      this->padding = padding;

      ROS_INFO_STREAM(name << " records bootstrap");

      DifferenceStream stream(param::difference_threshold(), path);
      for (int i = 0;; i++)
      {
        cv::Mat image = get(stream);
        if (image.empty())
          break;

        append(image);

        file_indexes << cv::Point(i, stream.index) << std::endl;
      }

      ROS_INFO_STREAM(name << " records bootstrap done (" << differences.size() << " entries)");
    }

    virtual void append(const cv::Mat &image)
    {
      differences.append(image);
      inverse_magnitudes.append(inverseMagnitudes(image, param::replay_padding()));
    }
  };

  struct Teach: public Session
  {
    int index;

    int range;

    typedef List<FeaturePoint> FeatureMap;

    List<FeatureMap> feature_maps;

    void load()
    {
      Session::load("teach", param::path_teach(), param::teach_padding());
      index = 0;
      range = differences.size();
    }

    virtual void append(const cv::Mat &image)
    {
      Session::append(image);
      int padding = param::replay_padding();
      int padding2 = param::teach_padding();
      List<FeaturePoint> features = selectSparseIntegrals(FEATURES, image, inverse_magnitudes[-1], padding, padding2);
      //spectra.append(spectralMap(features, image.size(), inverse_magnitudes[-1]));
      feature_maps.append(features);
    }
  };

  struct Replay: public Session
  {
    void load()
    {
      Session::load("replay", param::path_replay(), param::replay_padding());
    }

    virtual void append(const cv::Mat &image)
    {
      Session::append(image);
      spectra.append(fourier::transform(image));
    }
  };

  Teach teach;

  Replay replay;

  cv::Mat feature_responses;

  cv::Mat similarities;

  cv::Mat shifts;

  List<cv::Point2f> maxima;

  List<cv::Point> matches;

  boost::shared_ptr<cv::Point2f> line;

  double line_leak;

  std::ofstream file_similarities;

  std::ofstream file_shifts;

  std::ofstream file_matches;

  DiVS():
    line_leak(param::replay_leak()),
    file_similarities((ROOT + "similarities.txt").c_str()),
    file_shifts((ROOT + "shifts.txt").c_str()),
    file_matches((ROOT + "matches.txt").c_str())
  {
    teach.load();
    replay.load();
    feature_responses = cv::Mat(teach.differences.size(), FEATURES, CV_64F);
    similarities = cv::Mat(teach.differences.size(), replay.differences.size(), CV_64F, cv::Scalar::all(0));
    shifts = cv::Mat(replay.differences.size(), replay.differences[0].cols, CV_64F, cv::Scalar::all(0));
  }

  void relate(int j)
  {
    viewer::show("Replay", depths::bgr(replay.differences[j]));

    cv::Mat column_similarities(similarities, cv::Rect(j, 0, 1, similarities.rows));
    const cv::Mat &inverse_magnitudes = replay.inverse_magnitudes[j];
    const cv::Mat &spectra_j = replay.spectra[j];
    cv::Size size = inverse_magnitudes.size();

    for (int i = teach.index, n = i + teach.range; i < n; i++)
    {
      List<FeaturePoint> features = teach.feature_maps[i];
      for (int k = 0, z = features.size(); k < z; k++)
      {
        const FeaturePoint &point = features[k];
        const cv::Rect &bounds_reach = point.bounds_reach;
        const cv::Mat &spectra_k = point.spectra;

        cv::Mat correlated = fourier::multiply(spectra_j, spectra_k, true);
        cv::Mat responses = fourier::inverse(correlated, size).mul(inverse_magnitudes);

        cv::Point tl = clarus::argmax(responses(bounds_reach));
        int x = bounds_reach.x + tl.x;
        int y = bounds_reach.y + tl.y;

        int i_r = point.bounds_patch.y;
        int j_r = point.bounds_patch.x;
        double value = responses.at<double>(y, x) * teach.inverse_magnitudes[i].at<double>(i_r, j_r);

        column_similarities.at<double>(i, 0) += value;
      }
    }

    //addPoints(j);
    normalize_inline(column_similarities);
    showSimilarities();
  }
/*
  void relate(int j)
  {
    feature_responses = cv::Scalar(0);

    cv::Mat replayed = replay.differences[j];
    cv::Mat integral = replay.inverse_magnitudes[j];

    List<cv::Mat> patches, matches;
    List<FeaturePoint> features = selectSparseIntegrals(FEATURES, replayed, integral, replay.padding, teach.padding);
    for (int k = 0, m = features.size(); k < m; k++)
    {
      cv::Mat matched = relate(features[k], j, k, teach.index, teach.range);
      patches.append(features[k].patch);
      matches.append(matched);
    }

    addPoint(j);

    showMatches(j, patches, matches);
    showReplay(replayed, integral, features);
    showResponses();
    showSimilarities();
  }
*/
  cv::Mat relate(const FeaturePoint &point, int j, int k, int i0, int n)
  {
    cv::Mat column_responses(feature_responses, cv::Rect(k, i0, 1, n));
    int l = point.bounds_patch.width;
    cv::Mat match;
    double best = 0.0;


    for (int i = 0; i < n; i++)
    {
      int ik = i + i0;
      const cv::Mat &image = teach.differences[ik];

      const cv::Mat &spectra_j = point.spectra;
      cv::Mat neighborhood = images::convert(image(point.bounds_search), CV_64F);

      cv::Mat spectra_i = fourier::transform(neighborhood, 0, spectra_j.size());

      cv::Mat correction(teach.inverse_magnitudes[ik], point.bounds_reach);
      cv::Mat correlated = fourier::multiply(spectra_i, spectra_j, true);
      cv::Mat responses = fourier::inverse(correlated, point.reach).mul(correction);

      cv::Point tl = clarus::argmax(responses);
      int x = tl.x;
      int y = tl.y;

      cv::Mat matched = neighborhood(cv::Rect(x, y, l, l));

      int i_r = point.bounds_patch.y;
      int j_r = point.bounds_patch.x;
      double value = responses.at<double>(y, x) * replay.inverse_magnitudes[j].at<double>(i_r, j_r);
      similarities.at<double>(i0 + i, j) += value;
      column_responses.at<double>(i, 0) = value;

      if (value > best)
      {
        best = value;
        match = matched;
      }
    }

    normalize_inline(column_responses);

    return match;
  }

  void addPoints(int x)
  {
    int y0 = teach.index;
    cv::Mat column_similarities(similarities, cv::Rect(x, y0, 1, teach.range));

    List<int> indexes = argmax(column_similarities, TOP_POINTS);
    for (int i = 0, n = indexes.size(); i < n; i++)
      maxima.append(cv::Point2f(x, y0 + indexes[i]));

    file_similarities << similarities(cv::Rect(x, 0, 1, similarities.rows)).t() << std::endl;

    normalize_inline(column_similarities);

    if (matches.empty())
      matches.append(cv::Point(0, 0));
    else if (maxima.size() >= TOP_POINTS * WINDOW_X)
      fitLine(x);
  }

  void fitLine(int x)
  {
    cv::Point2f fitted = interpolateBest(maxima(-TOP_POINTS * WINDOW_X, 0), WINDOW_X - 2, 5.0);
    if (line.get() == NULL)
      line.reset(new cv::Point2f(fitted.x, fitted.y));
    else
    {
      line->x = line_leak * line->x + (1.0 - line_leak) * fitted.x;
      line->y = line_leak * line->y + (1.0 - line_leak) * fitted.y;
    }

    double b = line->x;
    double m = line->y;
    double y = x * m + b;
    cv::Point point(x, y);
    matches.append(point);
    file_matches << point << std::endl;

    teach.index = std::max(0.0, y - WINDOW_Y / 2);
    teach.range = std::min(WINDOW_Y, ((int) teach.differences.size()) - teach.index);

    cv::Mat replayed = replay.differences[x];
    cv::Mat teached = teach.differences[y];
    cv::Mat correlated = fourier::swap(fourier::correlate(teached, replayed, false));

    int y_c = correlated.rows / 2;
    int cols = correlated.cols;
    cv::Mat shift_vector(correlated, cv::Rect(0, y_c, cols, 1));
    cv::Mat shifts_row(shifts, cv::Rect(0, x, cols, 1));
    shift_vector.copyTo(shifts_row);

    file_shifts << shifts_row << std::endl;

    normalize_inline(shifts_row);

    viewer::show("Shifts", depths::bgr(shifts).t());
    viewer::show("Teach", depths::bgr(teached));
  }

  void showMatches(int index, const List<cv::Mat> &patches, const List<cv::Mat> &matches)
  {
    static int GAP = 3;
    static int SLOTS = 15;

    int n = patches.size();
    int w = patches[0].cols;
    int h = patches[0].rows;
    int rows = GAP + h * 2;
    int cols = std::min(SLOTS, n) * (w + GAP) - GAP;

    cv::Mat bgr(rows, cols, CV_8UC3, cv::Scalar::all(0));
    for (int i = 0; i < n; i++)
    {
      int j = i % SLOTS;
      depths::bgr(patches[i]).copyTo(bgr(cv::Rect(j * (w + GAP), 0, w, h)));

      const cv::Mat &match = matches[i];
      if (!match.empty())
        depths::bgr(match).copyTo(bgr(cv::Rect(j * (w + GAP), h + GAP, w, h)));

      int k = i + 1;
      if (k % SLOTS == 0 || k == n)
      {
        std::string part = types::to_string(k / SLOTS);
        viewer::show("Matches " + part, bgr);
        images::save(bgr, ROOT + "matches_" + types::to_string(index) + part + ".png");
        bgr = cv::Mat(rows, cols, CV_8UC3, cv::Scalar::all(0));
      }
    }
  }

  void showReplay(const cv::Mat &replayed, const cv::Mat &saliences, const List<FeaturePoint> &features)
  {
    cv::Mat bgr = depths::bgr(replayed);
    for (int i = 0, n = features.size(); i < n; i++)
    {
      const FeaturePoint &point = features[i];
      cv::rectangle(bgr, point.bounds_patch, WHITE);
      //cv::rectangle(bgr, point.bounds_search, BLACK);
      cv:putText(bgr, types::to_string(i), point.center, cv::FONT_HERSHEY_PLAIN, 1.0, WHITE);
    }

    viewer::show("Replay", bgr);
    viewer::show("Saliences", depths::bgr(-saliences));
  }

  void showResponses()
  {
    int rows = feature_responses.rows;
    int cols = feature_responses.cols;

    int xs = std::max(300.0 / cols, 3.0);
    int xn = cols * xs;
    int xl = xn - 1;
    int yl = rows - 1;

    cv::Mat bgr = depths::bgr(feature_responses, cv::Size(xn, rows));

    for (int i = xs - 1; i < xl; i += xs)
      cv::line(bgr, cv::Point(i, 0), cv::Point(i, yl), BLACK);

    viewer::show("Responses", bgr);
  }

  void showSimilarities()
  {
    cv::Mat similarities_bgr = depths::bgr(similarities);
    for (int j = 0, n = maxima.size(); j < n; j++)
    {
      cv::Point2f pj = maxima[j];
      int x = pj.x;
      int y = pj.y;

      cv::Vec3b &pixel = similarities_bgr.at<cv::Vec3b>(y, x);
      pixel[0] = 255;
      pixel[1] = 255;
      pixel[2] = 255;
    }

    for (int j = 1, n = matches.size(); j < n; j++)
      cv::line(similarities_bgr, matches[j - 1], matches[j], BLACK);

    viewer::show("Similarities", similarities_bgr);
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
