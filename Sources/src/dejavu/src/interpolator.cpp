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

#include <dejavu/interpolator.h>
using clarus::List;

#include <clarus/core/math.hpp>
#include <clarus/io/viewer.hpp>
#include <clarus/vision/images.hpp>

#include <cmath>

namespace dejavu
{

cv::Point lineP0(const cv::Point2f &line)
{
  float b = line.x;
  float m = line.y;
  return (b >= 0 ? cv::Point(0, b) : cv::Point(-b / m, 0));
}

cv::Point linePn(const cv::Point2f &line, const cv::Size &size)
{
  float b = line.x;
  float m = line.y;

  float xn = size.width - 1;
  float yn = xn * m + b;

  if (yn < size.height)
    return cv::Point(xn, yn);

  yn = size.height - 1;
  xn = (yn - b) / m;
  return cv::Point(xn, yn);
}

cv::Point interpolateParameters(cv::Mat &parameters, float bias, const cv::Mat &similarities)
{
  int rows = similarities.rows;
  int cols = similarities.cols;

  int b0 = rows / 2;
  int rows_p = rows * 10;

  if (parameters.empty())
    parameters = cv::Mat(rows_p, rows_p, CV_32F, cv::Scalar::all(0));
  else
    parameters *= bias;

  float best = 0.0;
  float b = 0.0;
  float m = 0.0;

  for (int i = 0; i < rows; i++)
  {
    for (int j = 0; j < cols; j++)
    {
      float value = similarities.at<float>(i, j);
      if (value == 0)
        continue;

      for (float m_k = 0.1; m_k < rows; m_k += 0.1)
      {
        float b_k = i - m_k * j;

        int u = 10 * m_k;
        int v = 10 * (b_k + b0);

        if (v < 0)
          break;
        if (v >= rows_p)
          continue;

        float &total = parameters.at<float>(u, v);
        total += value;
        if (total > best)
        {
          best = total;
          b = b_k;
          m = m_k;
        }
      }
    }
  }

  //viewer::plot("Parameters", parameters);

  return cv::Point(b, m);
}

cv::Point2f interpolateParameters(cv::Mat &parameters, float bias, float x, float y0, const cv::Mat &similarities)
{
  static float M = 2.5;
  static float S = 100.0;

  int n = similarities.rows;
  int cols = n * 2;
  int bk = cols / 2;
  int bn = cols - bk;

  if (parameters.empty())
    parameters = cv::Mat(M * S, cols, CV_32F, cv::Scalar::all(0));
  else
    parameters *= bias;

  float best = 0.0;
  cv::Point2f line(0, 0);

  for (float m = 0.0; m < M; m += 0.01)
  {
    float w = m * m + 1.0;
    for (int b = -bk; b < bn; b++)
    {
      float &total = parameters.at<float>(m * S, b + bk);
      for (int i = 0; i < n; i++)
      {
        float y = y0 + i;

        // Square of the the point-to-line equation
        // See: https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Line_defined_by_an_equation
        // a = m, b = -1, c = b (from y = mx + b -> mx -y + b = 0)
        float z = m * x - y + b;
        float d2 = (z * z) / w;
        if (d2 >= 1.0)
          continue;

        total += similarities.at<float>(i, 0);
      }

      if (total > best)
      {
        best = total;
        line.x = b;
        line.y = m;
      }
    }
  }

  return line;
}

cv::Point interpolateEnumerate(const cv::Mat &similarities)
{
  int rows = similarities.rows;
  int cols = similarities.cols;

  int b0 = rows / 2;
  int rows_p = rows * 10;
  cv::Mat parameters(rows_p, rows_p, CV_32F, cv::Scalar::all(0));

  float best = 0.0;
  float b = 0.0;
  float m = 0.0;

  for (int i = 0; i < rows; i++)
  {
    for (int j = 0; j < cols; j++)
    {
      float value = similarities.at<float>(i, j);
      if (value == 0)
        continue;

      for (float m_k = 0.1; m_k < rows; m_k += 0.1)
      {
        float b_k = i - m_k * j;

        int u = 10 * m_k;
        int v = 10 * (b_k + b0);

        if (v < 0)
          break;
        if (v >= rows_p)
          continue;

        float &total = parameters.at<float>(u, v);
        total += value;
        if (total > best)
        {
          best = total;
          b = b_k;
          m = m_k;
        }
      }
    }
  }

  //viewer::plot("Parameters", parameters);

  return cv::Point(b, m);
}

cv::Point2f interpolateFit(const List<cv::Point2f> &points, double m0, double mN)
{
  cv::Vec4f line;
  cv::fitLine(*points, line, CV_DIST_WELSCH, 0, 0.01, 0.01);

  float u = line[0];
  float v = line[1];
  float x = line[2];
  float y = line[3];

  float m = std::min(std::max(m0, u != 0 ? v / u : mN), mN);
  float b = y - m * x;
  return cv::Point2f(b, m);
}

cv::Point2f interpolateBest(const List<cv::Point2f> &points, double d_min, double error)
{
  cv::Point2f line(0, 0);
  int best = 0;

  int n = points.size();
  for (int i = 0; i < n; i++)
  {
    const cv::Point2f &p1 = points[i];
    double x1 = p1.x;
    double y1 = p1.y;

    for (int j = i + 1; j < n; j++)
    {
      const cv::Point2f &p2 = points[j];
      double x2 = p2.x;
      double y2 = p2.y;

      double dx = x2 - x1;
      double dy = y2 - y1;
      if (dx < d_min)
        continue;

      int count = 0;
      for (int k = 0; k < n; k++)
      {
        const cv::Point2f &p0 = points[k];
        double x0 = p0.x;
        double y0 = p0.y;

        double w = dy * x0 - dx * y0 + x2 * y1 - y2 * x1;
        double z = dy * dy + dx * dx;
        double d2 = w * w / z;
        if (d2 < error)
          count++;
      }

      if (count > best)
      {
        double m = dy / dx;
        double b = y1 - m * x1;

        best = count;
        line.x = b;
        line.y = m;
      }
    }
  }

  return line;
}

} // namespace dejavu
