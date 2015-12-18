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

#include <dejavu/file_stream.h>
#include <dejavu/filters.h>
#include <dejavu/dispatcher.h>
#include <dejavu/feature_selector.h>
#include <dejavu/interpolator.h>
#include <dejavu/settings.h>
#include <dejavu/working_memory.h>

#include <cv_video/settings.h>

#include <cv_video/video.h>
using cv_video::Video;
using cv_video::Frame;

#include <clarus/clarus.hpp>
using clarus::List;

#include <ros/console.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>

#include <ctime>
#include <iostream>
#include <fstream>

namespace dejavu
{

static const int KEY_ESC = 27;

static cv::Scalar WHITE(255, 255, 255);

template<class T>
inline T truncate(T value, T lower, T upper)
{
  return std::min(std::max(value, lower), upper);
}

static void normalize_inline(cv::Mat &data)
{
  data -= clarus::min(data);
  double w = clarus::max(data);
  if (w != 0)
    data /= w;
}

cv::Mat normalize(const cv::Mat &data)
{
  cv::Mat normalized = data.clone();
  normalize_inline(normalized);
  return normalized;
}

cv::Mat toRange(const cv::Mat &data, double alpha, double beta)
{
  cv::Mat ranged;
  cv::normalize(data, ranged, alpha, beta, CV_MINMAX);
  return ranged;
}

cv::Mat normalize(const cv::Mat &data, int x, int y, int width, int height)
{
  cv::Mat patch(data, cv::Rect(x, y, width, height));
  cv::Mat normalized = patch - clarus::min(patch);
  normalized /= clarus::max(normalized);
  return normalized;
}

cv::Point weightedAverage(const cv::Mat &data)
{
  if (data.type() != CV_64F)
    throw std::runtime_error("data.type() != CV_64F");

  double x = 0.0;
  double y = 0.0;
  double w = 0.0;

  int rows = data.rows;
  int cols = data.cols;
  for (int i = 0; i < rows; i++)
  {
    for (int j = 0; j < cols; j++)
    {
      double v = data.at<double>(i, j);
      x += j * v;
      y += i * v;
      w += v;
    }
  }

  return cv::Point(x / w, y / w);
}

float distance2(const cv::Point &a, const cv::Point &b)
{
  return pow(a.x - b.x, 2.0) + pow(a.y - b.y, 2.0);
}

cv::Mat drawRegions(cv::Mat &bgr)
{
/*
  static cv::Scalar BLACK(0, 0, 0);

  int width = bgr.cols;
  int width_side = width / 4;
  int height = bgr.rows / 2;

  cv::Rect left(0, 0, width_side, height);
  cv::Rect right(width - width_side, 0, width_side, height);
  cv::Rect top(left.width, 0, 2 * width_side, height / 2);
  cv::Rect center(left.width, top.height, width / 2, height / 2);

  cv::rectangle(bgr, left, BLACK);
  cv::rectangle(bgr, right, BLACK);
  cv::rectangle(bgr, top, BLACK);
  cv::rectangle(bgr, center, BLACK);
*/
  return bgr;
}

class MatchingInspector
{
  struct PointComp
  {
    bool operator () (const cv::Point3f &a, const cv::Point3f &b)
    {
      return (a.z < b.z);
    }
  };

  struct Best
  {
    int size;

    std::set<cv::Point3f, PointComp> buffer;

    Best(int n):
      size(n),
      buffer()
    {
      // nothing to do.
    }

    void add(int i, int j, float value)
    {
      cv::Point3f point(j, i, value);
      buffer.insert(point);
      if (buffer.size() > size)
        buffer.erase(buffer.begin());
    }

    void dump(WorkingMemory<cv::Point2f> &memory)
    {
      for (std::set<cv::Point3f>::const_iterator i = buffer.begin(), n = buffer.end(); i != n; ++i)
      {
        const cv::Point3f &point = *i;
        memory.append(cv::Point2f(point.x, point.y));
      }
    }

    void dump(cv::Mat &data)
    {
      for (std::set<cv::Point3f>::const_iterator i = buffer.begin(), n = buffer.end(); i !=n; ++i)
      {
        const cv::Point3f &point = *i;
        data.at<float>(point.y, point.x) = point.z;
      }
    }

    cv::Point2f average()
    {
      float value = 0;
      float x = 0;
      float y = 0;

      for (std::set<cv::Point3f>::const_iterator i = buffer.begin(), n = buffer.end(); i !=n; ++i)
      {
        const cv::Point3f &point = *i;
        float w = point.z;

        x += point.x * w;
        y += point.y * w;
        value += w;
      }

      x /= value;
      y /= value;

      return cv::Point2f(x, y);
    }

    void average(cv::Mat &data)
    {
      float value = 0;
      float x = 0;
      float y = 0;

      for (std::set<cv::Point3f>::const_iterator i = buffer.begin(), n = buffer.end(); i !=n; ++i)
      {
        const cv::Point3f &point = *i;
        float w = point.z;

        x += point.x * w;
        y += point.y * w;
        value += w;
      }

      x /= value;
      y /= value;

      data.at<float>(y, x) = value;
    }
  };

  struct Session
  {
    List<cv::Mat> images;

    List<cv::Mat> grayscale;

    List<cv::Mat> saliences;

    List<cv::Mat> filtered;

    List<cv::Mat> spectra;

    Session(const std::string &name, const std::string &path)
    {
      ROS_INFO_STREAM(name << " records bootstrap");

      ros::spinOnce();

      int index = 1;
      cv::Mat saliences;
      for (FileStream stream(0, path); ros::ok();)
      {
        cv::Mat image = stream();
        if (image.empty())
          break;

        cv::Mat scaled = images::scale(image, 256);
        cv::Mat grays = colors::grayscale(scaled);
        if (cv::sum(grays - clarus::mean(grays))[0] == 0)
          continue;

        if (saliences.empty())
          saliences = saliencyMap(scaled);
        else
          saliences += saliencyMap(scaled);

        if (index % 5 == 0)
        {
          cv::Mat spatial = images::convert(grays, CV_64F).mul(saliences);
          cv::Mat frequency = fourier::transform(fourier::normalize(spatial));

          images.append(scaled);
          grayscale.append(grays);
          this->saliences.append(saliences.clone());
          filtered.append(spatial);
          spectra.append(frequency);

          saliences = cv::Scalar::all(0);
/*
          if (spectra.size() > 1)
          {
            cv::Mat correlated = fourier::multiply(spectra[-2], spectra[-1], true);
            cv::Mat responses = depths::bgr(fourier::inverse(correlated, spatial.size()));
            viewer::plot(name + " fourier difference", fourier::swap(responses));
          }

          viewer::show(name + " image", depths::bgr(spatial));
*/
        }

        index++;
      }

      ROS_INFO_STREAM(name << " records bootstrap done (size: " << filtered.size() << ")");

      ros::spinOnce();
    }
  };

  struct Teach: public Session
  {
    int index;

    int reach;

    int padding;

    float leak;

    boost::shared_ptr<cv::Point2f> matching;

    Teach():
      Session("Teach", param::path_teach()),
      index(0),
      reach(param::teach_images()),
      padding(param::teach_padding()),
      leak(0.98)
    {
      // Nothing to do.
    }

    bool done()
    {
      return (index + param::teach_images() >= images.size());
    }

    cv::Mat image(int j)
    {
      int k = imageIndex(j);
      if (k < 0)
        return filtered[0];
      if (k >= filtered.size())
        return filtered[-1];

      return filtered[k];
    }

    inline int imageIndex(int j)
    {
      return matching->y * j + matching->x;
    }

    void update(float &estimate, float value)
    {
      estimate = leak * estimate + (1.0 - leak) * value;
    }

    void update(const cv::Point2f &line)
    {
      if (matching.get() == NULL)
        matching.reset(new cv::Point2f(line.x, line.y));
      else
      {
        update(matching->x, line.x);
        update(matching->y, line.y);
      }

      ROS_INFO_STREAM("Line: " << *matching);
    }
  };

  struct Replay: public Session
  {
    int index;

    int reach;

    int padding;

    Replay():
      Session("Replay", param::path_replay()),
      index(0),
      reach(param::replay_images()),
      padding(param::replay_padding())
    {
      // Nothing to do.
    }

    List<cv::Mat> operator() ()
    {
      cv::Mat image = filtered[index];
      cv::Mat frequency = spectra[index];
      index++;

      return (List<cv::Mat>(), image, frequency);
    }
  };

  struct Similarities: public cv::Mat
  {
    Teach teach;

    Replay replay;

    cv::Mat parameters;

    cv::Mat shift_map;

    cv::Mat similarities;

    cv::Mat stdev;

    WorkingMemory<cv::Point2f> matches;

    bool display;

    std::ofstream file_shifts;

    std::ofstream file_similarities;

    std::ofstream file_matching;

    Similarities():
      matches(param::replay_images()),
      display(false),
      file_shifts("/home/helio/Roboken/Projects/Active/Lividum/shifts.txt"),
      file_similarities("/home/helio/Roboken/Projects/Active/Lividum/similarities.txt"),
      file_matching("/home/helio/Roboken/Projects/Active/Lividum/matching.txt")
    {
      cv::Mat &data = *this;
      data = cv::Mat(teach.images.size(), replay.images.size(), CV_32F, cv::Scalar(0));
      similarities = cv::Mat(teach.images.size(), replay.images.size(), CV_32F, cv::Scalar(0));
      stdev = cv::Mat(teach.images.size(), replay.images.size(), CV_32F, cv::Scalar(0));
      shift_map = cv::Mat(replay.images.size(), replay.images[0].cols, CV_32F, cv::Scalar(0));
    }

    void analyse(int i, const cv::Mat &responses)
    {
      cv::Mat data = fourier::swap(normalize(responses));
      cv::Point max = clarus::argmax(data);
      cv::Point avg = weightedAverage(data);

      viewer::show("Similarity teach", depths::bgr(teach.filtered[i]));
      viewer::show("Similarity fourier", depths::bgr(data));

      ROS_INFO_STREAM("Distance " << max << " - " << avg << " = " << distance2(avg, max));

      cv::waitKey();
    }

    cv::Point2d similarity(const cv::Mat &responses)
    {
      cv::Vec2d std_err;
      cv::Mat data = fourier::swap(responses);
      cv::meanStdDev(normalize(data), cv::noArray(), std_err);
/*
      cv::Mat data = fourier::swap(responses);
      data -= clarus::min(data);

      cv::Point max = clarus::argmax(data);
      cv::Point avg = weightedAverage(data);

      if (distance2(avg, max) > 500)
        return 0;
      else
        return data.at<double>(max.y, max.x);
*/
      int rows = responses.rows;
      int cols = responses.cols;
      int w = cols / 2;
      int h = 6;
      int x = cols / 4;
      int y = (rows - h) / 2;
      return cv::Point2d(clarus::max(data(cv::Rect(x, y, w, h))), std_err[0]);
    }

    void relate()
    {
      int j = replay.index;
      List<cv::Mat> frames = replay();
      const cv::Mat image = frames[0];
      const cv::Mat spectra = frames[1];

      viewer::show("Replay image", depths::bgr(image));

      Best best(10);
      cv::Size size = fourier::fit(teach.images[0].size());
      for (int i = 0, n = teach.images.size(); i < n; i++)
      {
        cv::Mat correlated = fourier::multiply(teach.spectra[i], spectra, true);
        cv::Mat responses = fourier::inverse(correlated, size);

        cv::Point2d s = similarity(responses);
        double value = s.x;
        double error = s.y;
        if (value == 0)
          continue;

        //analyse(i, responses);

        //double value = clarus::max(responses(cv::Rect(0, 0, responses.cols, 1)));
        similarities.at<float>(i, j) = value;
        stdev.at<float>(i, j) = error;
        best.add(i, j, value);
      }

      matches.append(best.average());
      if (matches.full())
      {
        teach.update(interpolateFit(*matches));
        estimate(j, size);
      }

      cv::Rect roi(j, 0, 1, rows);

      cv::Mat column_similarities(similarities, roi);
      cv::Mat column_stdev(stdev, roi);
      cv::Mat column(*this, roi);

      column_similarities.copyTo(column);

      normalize_inline(column_similarities);
      normalize_inline(column_stdev);
      normalize_inline(column);

      showSimilarityMap(j);

      file_similarities << column.t() << std::endl;
    }

    void estimate(int j, const cv::Size &size)
    {
      int k = truncate(teach.imageIndex(j), 0, (int) teach.spectra.size() - 1);
      cv::Mat correlated = fourier::multiply(replay.spectra[j], teach.spectra[k], true);
      cv::Mat responses = fourier::swap(fourier::inverse(correlated, size));

      cv::Point max = clarus::argmax(responses);
      cv::Mat shifts = normalize(responses, 0, max.y, responses.cols, 1);

      file_shifts << shifts << std::endl;

      shifts.copyTo(shift_map(cv::Rect(0, shift_map.rows - j, shift_map.cols, 1)));

      cv::Mat shift_map_bgr = depths::bgr(shift_map);
      int yn = shift_map_bgr.rows - 1;
      int x = shift_map_bgr.cols / 2;

      cv::line(shift_map_bgr, cv::Point(x, 0), cv::Point(x, yn), WHITE);

      viewer::show("Shifts", shift_map_bgr);

      cv::Mat fourier_difference = depths::bgr(normalize(responses));
      viewer::show("Fourier difference", fourier_difference);
/*
      std::string folder("/home/helio/Roboken/Projects/Active/Lividum/");

      images::save(fourier_difference, folder + "fourier_difference.png");

      images::save(replay.images[j], folder + "replay_image.png");
      images::save(replay.grayscale[j], folder + "replay_grayscale.png");
      images::save(depths::bgr(normalize(replay.saliences[j])), folder + "replay_saliences.png");
      images::save(replay.filtered[j], folder + "replay_filtered_grayscale.png", true);
      images::save(depths::bgr(replay.filtered[j]), folder + "replay_filtered.png");

      images::save(teach.images[k], folder + "teach_image.png");
      images::save(teach.grayscale[k], folder + "teach_grayscale.png");
      images::save(depths::bgr(normalize(teach.saliences[k])), folder + "teach_saliences.png");
      images::save(teach.filtered[j], folder + "teach_filtered_grayscale.png", true);
      images::save(depths::bgr(teach.filtered[k]), folder + "teach_filtered.png");

      cv::waitKey();
*/
    }

    void showSimilarityMap(int j)
    {
      cv::Mat bgr = depths::bgr(*this);
      if (matches.full())
      {
        int x0 = j + 1 - matches.size();
        int y0 = truncate(teach.imageIndex(x0), 0, rows);
        int yn = truncate(teach.imageIndex(j), 0, rows);

        cv::Point p0(x0, y0);
        cv::Point pn(j, yn);

        file_matching << pn << std::endl;

        cv::line(bgr, p0, pn, WHITE);

        viewer::show("Teach image", depths::bgr(teach.image(j)));

        ROS_INFO_STREAM("Draw: " << p0 << ", " << pn);
      }

      viewer::show("Similarity map", bgr);
      viewer::show("Similarity map (raw)", depths::bgr(similarities));
      viewer::show("Standard deviation", depths::bgr(stdev));
    }

    void spin()
    {
      for (int j = 0, n = replay.images.size(); j < n && ros::ok(); j++)
      {
        relate();
        ros::spinOnce();
      }

      while (ros::ok())
      {
        int key = cv::waitKey(20);
        if (key == KEY_ESC)
          return;

        ros::spinOnce();
      }
    }
  };

public:
  Similarities similarities;

  MatchingInspector()
  {
    // Nothing to do.
  }

  void spin()
  {
    similarities.spin();
  }
};

void inspector_matching(int argc, char *argv[])
{
  ros::init(argc, argv, "inspector_matching");

  MatchingInspector inspector;

  inspector.spin();
}

} // namespace dejavu

int main(int argc, char *argv[])
{
  dejavu::inspector_matching(argc, argv);

  return 0;
}
