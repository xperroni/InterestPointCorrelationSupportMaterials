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
#include <dejavu/working_memory.h>

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

static cv::Scalar ZERO = cv::Scalar::all(0);

static cv::Scalar BLACK = ZERO;

static cv::Scalar WHITE(255, 255, 255);

static double l_2PS = 1.0 / sqrt(2 * M_PI);

static int FEATURES = 30;

static int WINDOW_X = 40;

static int WINDOW_Y = 50;

static int TOP_POINTS = 5;

static void gaussian(double value, int i, cv::Mat &column)
{
  if (value == 0.0)
    return;

  double a = std::max(i - 4.0, 0.0);
  double n = std::min(i + 4.0, (double) column.rows);
  for (double k = a; k < n; k+= 1.0)
  {
    double u = k - i;
    column.at<double>(k, 0) += value * l_2PS * exp(-0.5 * u * u);
  }
}

static void normalize_inline(cv::Mat &data)
{
  data -= clarus::min(data);
  double w = clarus::max(data);
  if (w != 0)
    data /= w;
}

static cv::Mat inverseMagnitudes(const cv::Mat &integral2, int padding)
{
  int side = 2 * padding + 1;
  cv::Rect area(0, 0, side, side);

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

template<class T>
inline T truncate(T value, T lower, T upper)
{
  return std::min(std::max(value, lower), upper);
}

static cv::Mat normalizeDifferences(const cv::Mat &differences, const cv::Mat &inverse_magnitudes, int padding)
{
  cv::Mat normalized(differences.size(), CV_64F);
  int rows = differences.rows;
  int cols = differences.cols;
  int u_n = inverse_magnitudes.rows - 1;
  int v_n = inverse_magnitudes.cols - 1;
  for (int i = 0; i < rows; i++)
  {
    for (int j = 0; j < cols; j++)
    {
      int u = truncate(i - padding, 0, u_n);
      int v = truncate(j - padding, 0, v_n);
      normalized.at<double>(i, j) = differences.at<double>(i, j) * inverse_magnitudes.at<double>(u, v);
    }
  }

  return normalized;
}

static cv::Mat get(DifferenceStream &stream, int skip = 4)
{
  for (int i = 0; i < skip; i++)
    stream();

  cv::Mat image = stream();
  if (image.empty())
    return cv::Mat();

  return images::scale(image, 256);
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
    std::string name;

    List<cv::Mat> differences;

    List<cv::Mat> integrals;

    List<cv::Mat> inverse_magnitudes;

    int padding;

    void load(const std::string &name, const std::string &path, int padding)
    {
      this->name = name;
      this->padding = padding;

      std::ofstream file_indexes((ROOT + "indexes_" + name + ".txt").c_str());

      ROS_INFO_STREAM(name << " records bootstrap");

      DifferenceStream stream(param::difference_threshold(), path);
      for (int i = 0;; i++)
      {
        cv::Mat image = get(stream);
        if (image.empty())
          break;

        append(i, image);

        file_indexes << cv::Point(i, stream.index) << std::endl;
      }

      ROS_INFO_STREAM(name << " records bootstrap done (" << differences.size() << " entries)");
    }

    virtual void append(int i, const cv::Mat &image)
    {
      cv::Mat image2 = image.mul(image);
      integrals.append(images::integral(image2));
      inverse_magnitudes.append(inverseMagnitudes(integrals[-1], param::replay_padding()));
      differences.append(normalizeDifferences(image, inverse_magnitudes[-1], param::replay_padding()));

      images::save(depths::bgr(image), ROOT + name + "_bgr_" + types::to_string(i) + ".png");
      images::save(colors::discrete(image), ROOT + name + "_grays_" + types::to_string(i) + ".png");

      images::save(depths::bgr(differences[-1]), ROOT + name + "_bgr_normalized_" + types::to_string(i) + ".png");
      images::save(colors::discrete(differences[-1]), ROOT + name + "_grays_normalized_" + types::to_string(i) + ".png");
    }
  };

  struct Teach: public Session
  {
    int index;

    int range;

    void load()
    {
      Session::load("teach", param::path_teach(), param::teach_padding());
      index = 0;
      range = differences.size();
    }

    virtual void append(int i, const cv::Mat &image)
    {
      Session::append(i, image);
    }
  };

  struct Replay: public Session
  {
    void load()
    {
      Session::load("replay", param::path_replay(), param::replay_padding());
    }

    virtual void append(int i, const cv::Mat &image)
    {
      Session::append(i, image);
      images::save(depths::bgr(image), ROOT + "replay_" + types::to_string(i) + ".png");
    }
  };

  Teach teach;

  Replay replay;

  cv::Mat feature_responses;

  cv::Mat similarities;

  WorkingMemory<cv::Mat> correlations;

  cv::Mat shift_estimate;

  cv::Mat shifts;

  List<cv::Point2f> maxima;

  List<cv::Point> matches;

  boost::shared_ptr<cv::Point2f> line;

  double line_leak;

  std::ofstream file_similarities;

  std::ofstream file_correlations;

  std::ofstream file_shifts;

  std::ofstream file_matches;

  DiVS():
    correlations(param::replay_images()),
    line_leak(param::replay_leak()),
    file_similarities((ROOT + "similarities.txt").c_str()),
    file_correlations((ROOT + "correlations.txt").c_str()),
    file_shifts((ROOT + "shifts.txt").c_str()),
    file_matches((ROOT + "matches.txt").c_str())
  {
    teach.load();
    replay.load();

    int rows = teach.differences.size();
    int cols = replay.differences.size();
    feature_responses = cv::Mat(rows, FEATURES, CV_64F);
    similarities = cv::Mat(rows, cols, CV_64F, ZERO);
    shifts = cv::Mat(cols, replay.differences[0].cols, CV_64F, ZERO);
  }

  void relate(int j)
  {
    feature_responses = ZERO;

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
    showReplay(j, replayed, features);
    showResponses();
    showSimilarities();
  }

  cv::Mat relate(const FeaturePoint &point, int j, int k, int i0, int n)
  {
    cv::Mat column_responses(feature_responses, cv::Rect(k, i0, 1, std::min(n, feature_responses.rows - i0)));
    //cv::Mat column_similarities(similarities, cv::Rect(j, i0, 1, n));
    int l = point.bounds_patch.width;
    double best = 0.0;
    int best_i = 0;
    cv::Mat match;

    const cv::Mat &spectra_j = point.spectra;
    const cv::Rect &bounds_patch = point.bounds_patch;
    const cv::Rect &bounds_reach = point.bounds_reach;
    const cv::Rect &bounds_search = point.bounds_search;

    for (int i = 0; i < n; i++)
    {
      int ik = i + i0;
      const cv::Mat &image = teach.differences[ik];

      cv::Mat neighborhood = image(bounds_search).clone();
      cv::Mat spectra_i = fourier::transform(neighborhood);
      cv::Mat correlated = fourier::multiply(spectra_i, spectra_j, true);
      cv::Mat responses = fourier::inverse(correlated, point.reach);

      cv::Point tl = clarus::argmax(responses);
      double value = responses.at<double>(tl.y, tl.x);

      similarities.at<double>(ik, j) += value;
      column_responses.at<double>(i, 0) = value;
      //gaussian(value, i, column_similarities);

      if (value > best)
      {
        best_i = i;
        best = value;
        int x0 = bounds_reach.x + tl.x;
        int y0 = bounds_reach.y + tl.y;
        match = image(cv::Rect(x0, y0, l, l));
      }
    }

    normalize_inline(column_responses);

    return match;
  }

  void addPoint(int x)
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

    //teach.range = std::min(WINDOW_Y * (1.0 + fabs(m)), (double) teach.differences.size());
    teach.range = WINDOW_Y; // * (1.0 + fabs(m));
    teach.index = truncate(y - teach.range / 2, 0.0, (double) (teach.differences.size() - teach.range));

    estimateShifts(y, x);
  }

  void estimateShifts(int i, int j)
  {
    cv::Mat replayed = replay.differences[j];
    cv::Mat teached = teach.differences[truncate(i, 0, (int) teach.differences.size() - 1)];

    int rows = teached.rows;
    int cols = teached.cols;

    int slit = param::divs_width();
    int slide = cols - slit;

    cv::Mat responses(1, slide * 2, CV_64F, cv::Scalar::all(0));
    for (int i = 0; i <= slide; i+= slit) {
        cv::Mat slice(teached, cv::Rect(i, 0, slit, rows));
        cv::Mat correlated = fourier::correlate(replayed, slice);
        cv::Mat shifts(correlated, cv::Rect(0, 0, slide, 1));

        cv::Mat range(responses, cv::Rect(slide - i, 0, slide, 1));
        range += shifts;
    }

    if (shift_estimate.empty())
      shift_estimate = cv::Mat(1, cols, CV_64F, cv::Scalar::all(0));

    cv::Mat correlation(responses, cv::Rect(slide - cols / 2, 0, cols, 1));
    if (!correlations.idle())
      shift_estimate -= correlations[0];

    file_correlations << correlation << std::endl;

    correlations.append(correlation);
    shift_estimate += correlation;

    file_shifts << shift_estimate << std::endl;

    cv::Mat shifts_row(shifts, cv::Rect(0, j, cols, 1));
    shift_estimate.copyTo(shifts_row);

    normalize_inline(shifts_row);

    int x2 = shifts.cols / 2;
    int yn = shifts.rows;
    cv::Mat shifts_bgr = depths::bgr(shifts);
    cv::line(shifts_bgr, cv::Point(x2, 0), cv::Point(x2, yn), WHITE);

    viewer::show("Shifts", shifts_bgr.t());
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

    cv::Mat bgr(rows, cols, CV_8UC3, ZERO);
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
        //images::save(bgr, ROOT + "matches_" + types::to_string(index) + part + ".png");
        bgr = cv::Mat(rows, cols, CV_8UC3, ZERO);
      }
    }
  }

  void showReplay(int j, const cv::Mat &replayed, const List<FeaturePoint> &features)
  {
    cv::Mat bgr = depths::bgr(replayed);

    for (int i = 0, n = features.size(); i < n; i++)
    {
      const FeaturePoint &point = features[i];
      cv::rectangle(bgr, point.bounds_patch, WHITE);
      //cv::rectangle(bgr, point.bounds_search, BLACK);
      //cv:putText(bgr, types::to_string(i), point.center, cv::FONT_HERSHEY_PLAIN, 1.0, WHITE);
    }

    viewer::show("Replay", bgr);
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
