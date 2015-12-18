/*
Copyright (c) Helio Perroni Filho <xperroni@gmail.com>

This file is part of Cight.

Cight is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

Cight is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Cight. If not, see <http://www.gnu.org/licenses/>.
*/

#include <dejavu/feature_selector.h>

#include <dejavu/filters.h>

#include <clarus/clarus.hpp>
using clarus::List;
using clarus::Point;

#include <clarus/vision/colors.hpp>
#include <clarus/vision/filters.hpp>
#include <clarus/vision/images.hpp>

namespace dejavu
{

List<FeaturePoint> selectSparseIntegrals(int limit, const cv::Mat &image, const cv::Mat &integral, int padding, int padding2)
{
  static cv::Scalar BLACK(0);
  static cv::Scalar WHITE(1);

  cv::Mat saliences = colors::discrete(integral);
  int rows = saliences.rows;
  int cols = saliences.cols;

  // Order saliences points by intensity
  List<List<cv::Point2i> > points(255);
  for (int i = 0; i < rows; i++)
  {
    for (int j = 0; j < cols; j++)
    {
      int k = saliences.at<uint8_t>(i, j);
      if (k == 255)
        continue;

      cv::Point2i &point = points[k].append();
      point.y = i;
      point.x = j;
    }
  }

  int side = 2 * padding + 1;

  cv::Mat active(saliences.size(), CV_8U, BLACK);
  List<FeaturePoint> features;
  int selected = 0;

  // Select disjoint feature points
  for (int i = 0, m = points.size(); i < m; i++)
  {
    List<cv::Point2i> &range = points[i];
    for (int j = 0, n = range.size(); j < n; j++)
    {
      cv::Point2i &point = range[j];
      int x = point.x;
      int y = point.y;
      if (active.at<uint8_t>(y, x) == 1)
        continue;

      features.append(FeaturePoint(x, y, image, padding, padding2));
      if (++selected == limit)
        return features;

      int x1 = std::max(x - side, 0);
      int y1 = std::max(y - side, 0);
      int x2 = std::min(x + side, cols);
      int y2 = std::min(y + side, rows);

      int width = x2 - x1;
      int height = y2 - y1;

      cv::rectangle(active, cv::Rect(x1, y1, width, height), WHITE, CV_FILLED);
    }
  }

  return features;
}

List<FeaturePoint> selectSparseDifferences(int limit, const cv::Mat &image, int padding, int padding2)
{
  static cv::Scalar BLACK(0);
  static cv::Scalar WHITE(1);

  int rows = image.rows;
  int cols = image.cols;

  // Order image points by intensity
  List<List<cv::Point2i> > points(255);
  for (int i = 0; i < rows; i++)
  {
    for (int j = 0; j < cols; j++)
    {
      int k = image.at<uint8_t>(i, j);
      if (k == 0)
        continue;

      cv::Point2i &point = points[255 - k].append();
      point.y = i;
      point.x = j;
    }
  }

  int padding2t = 2 * padding;
  int side2t = 2 * padding2t + 1;

  cv::Mat active(image.size(), CV_8U, BLACK);
  List<FeaturePoint> features;
  int selected = 0;

  // Select disjoint feature points
  for (int i = 0, m = points.size(); i < m; i++)
  {
    List<cv::Point2i> &range = points[i];
    for (int j = 0, n = range.size(); j < n; j++)
    {
      cv::Point2i &point = range[j];
      int x = point.x;
      int y = point.y;

      int u = y + std::max(padding - y, 0) + std::min(rows - y - padding, 0);
      int v = x + std::max(padding - x, 0) + std::min(cols - x - padding, 0);
      if (active.at<uint8_t>(u, v) == 1)
        continue;

      features.append(FeaturePoint(x, y, image, padding, padding2));
      if (++selected == limit)
        return features;

      int x2 = std::min(std::max(v - padding2t, 0), cols - side2t);
      int y2 = std::min(std::max(u - padding2t, 0), rows - side2t);
      cv::rectangle(active, cv::Rect(x2, y2, side2t, side2t), WHITE, CV_FILLED);
    }
  }

  return features;
}

List<FeaturePoint> selectSparseSaliences(int limit, const cv::Mat &image, int padding, int padding2)
{
  static cv::Scalar BLACK(0);
  static cv::Scalar WHITE(1);

  int rows = image.rows;
  int cols = image.cols;

  cv::Mat saliences = colors::discrete(saliencyMap(image));
  cv::Mat grays = colors::grayscale(image);

  // Order image points by intensity
  List<List<cv::Point2i> > points(255);
  for (int i = 0; i < rows; i++)
  {
    for (int j = 0; j < cols; j++)
    {
      int k = saliences.at<uint8_t>(i, j);
      if (k == 0)
        continue;

      cv::Point2i &point = points[255 - k].append();
      point.y = i;
      point.x = j;
    }
  }

  int padding2t = 2 * padding;
  int side2t = 2 * padding2t + 1;

  cv::Mat active(image.size(), CV_8U, BLACK);
  List<FeaturePoint> features;
  int selected = 0;

  // Select disjoint feature points
  for (int i = 0, m = points.size(); i < m; i++)
  {
    List<cv::Point2i> &range = points[i];
    for (int j = 0, n = range.size(); j < n; j++)
    {
      cv::Point2i &point = range[j];
      int x = point.x;
      int y = point.y;

      int u = y + std::max(padding - y, 0) + std::min(rows - y - padding, 0);
      int v = x + std::max(padding - x, 0) + std::min(cols - x - padding, 0);
      if (active.at<uint8_t>(u, v) == 1)
        continue;

      features.append(FeaturePoint(x, y, grays, padding, padding2));
      if (++selected == limit)
        return features;

      int x2 = std::min(std::max(v - padding2t, 0), cols - side2t);
      int y2 = std::min(std::max(u - padding2t, 0), rows - side2t);
      cv::rectangle(active, cv::Rect(x2, y2, side2t, side2t), WHITE, CV_FILLED);
    }
  }

  return features;
}

} // namespace dejavu
