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

#include <dejavu/feature_point.h>
using clarus::List;

namespace dejavu
{

FeaturePoint::FeaturePoint():
  bounds_patch(0, 0, 0, 0),
  center(0, 0),
  patch(),
  reach(),
  spectra()
{
    // Nothing to do.
}

inline cv::Rect regionOfInterest(int x, int y, const cv::Mat &image, int padding)
{
  int side = 2 * padding + 1;
/*
  int xf = std::min(std::max(0, x - padding), image.cols - side);
  int yf = std::min(std::max(0, y - padding), image.rows - side);
  return cv::Rect(xf, yf, side, side);
*/

  return cv::Rect(x, y, side, side);
}

inline cv::Rect regionOfSearch(const cv::Rect &bounds_patch, const cv::Mat &image, int padding)
{
  int w = bounds_patch.width + 2 * padding;
  int h = 2 * bounds_patch.height;
  int x = std::min(std::max(0, bounds_patch.x - padding), image.cols - w);
  int y = std::min(std::max(0, bounds_patch.y - bounds_patch.height / 2), image.rows - h);
  return cv::Rect(x, y, w, h);
}

inline cv::Rect regionOfReach(const cv::Rect &bounds_patch, const cv::Rect &bounds_search)
{
  int x = bounds_search.x;
  int y = bounds_search.y;
  int w = bounds_search.width  - bounds_patch.width;
  int h = bounds_search.height - bounds_patch.height;
  return cv::Rect(x, y, w, h);
}

inline float standardDeviation(const cv::Mat &patch)
{
  cv::Mat mean, stddev;
  cv::meanStdDev(patch, mean, stddev);
  return stddev.at<double>(0, 0);
}

inline cv::Mat patchSpectra(const cv::Mat &patch, const cv::Rect &bounds_search)
{
/*
  cv::Mat normalized = images::convert(patch, CV_64F);
  double magnitude = sqrt(cv::sum(normalized.mul(normalized))[0]);
  normalized *= (1.0 / magnitude);

  return fourier::transform(normalized, 0, fourier::fit(side, side));
*/

  //cv::Mat normalized = images::convert(patch, CV_64F);
  //normalized -= cv::mean(normalized)[0];

  return fourier::transform(patch, 0, fourier::fit(bounds_search.width, bounds_search.height));
}

FeaturePoint::FeaturePoint(int x, int y, const cv::Mat &image, int padding, int padding2):
  bounds_patch(regionOfInterest(x, y, image, padding)),
  bounds_search(regionOfSearch(bounds_patch, image, padding2)),
  bounds_reach(regionOfReach(bounds_patch, bounds_search)),
  center(bounds_patch.x + bounds_patch.width / 2, bounds_patch.y + bounds_patch.height / 2),
  patch(image(bounds_patch).clone()),
  reach(bounds_reach.width, bounds_reach.height),
  spectra(patchSpectra(patch, bounds_search)),
  strength(standardDeviation(patch))
{
  // Nothing to do.
}

int FeaturePoint::operator () (const List<cv::Mat> &images, int i0, int n) const
{
  int index = 0;
  double best = 0.0;
  for (int i = 0; i < n; i++)
  {
    const cv::Mat &image = images.at(i + i0);

    cv::Mat neighborhood(image, bounds_search);
    cv::Mat spectra_i = fourier::transform(fourier::normalize(neighborhood), 0, spectra.size());

    cv::Mat correlated = fourier::multiply(spectra_i, spectra, true);
    cv::Mat responses = fourier::inverse(correlated, reach);
    double value = clarus::max(responses);
    if (value > best)
    {
      best = value;
      index = i;
    }
  }

  return index;
}

} // namespace dejavu
