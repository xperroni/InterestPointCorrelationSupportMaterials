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

#include <dejavu/filters.h>

#include <clarus/clarus.hpp>
using clarus::List;

namespace dejavu
{

cv::Mat saliencyMap(const cv::Mat &frame)
{
  static cv::Mat h_nf(3, 3, CV_64F, cv::Scalar(1.0 / 9.0));

  if (frame.channels() > 1)
  {
    List<cv::Mat> channels = clarus::split(frame);
    cv::Mat summed(frame.size(), CV_64F, cv::Scalar(0));
    for (int i = 0, n = channels.size(); i < n; i++)
      summed += saliencyMap(channels[i]);

    return summed;
  }

  cv::Mat I = images::convert(frame, CV_64F);
  cv::Mat F = fourier::transform(I, cv::DFT_COMPLEX_OUTPUT);
  cv::Mat Af = fourier::magnitude(F);
  cv::Mat Pf = fourier::phase(F);
  cv::Mat Lf = clarus::log(Af);
  cv::Mat Rf = Lf - fourier::convolve(Lf, h_nf);
  cv::Mat Ef = clarus::exp(Rf, Pf);
  cv::Mat Sf = clarus::pow(fourier::inverse(Ef), 2.0);

  return clarus::split(Sf)[0];
}

cv::Mat saliencyMap(const cv::Mat &frame, int width)
{
  cv::Size original = frame.size();

  double factor = ((double) width) / ((double) original.width);
  cv::Size scale(width, factor * original.height);

  cv::Mat scaled_frame = images::scale(frame, scale);
  cv::Mat saliences = saliencyMap(scaled_frame);
  return images::scale(saliences, original);
  //return saliences;
}

cv::Mat upperHalf(const cv::Mat &image)
{
  cv::Rect roi(0, 0, image.cols, image.rows / 2);
  cv::Mat half(image, roi);
  return half;
}

} // namespace dejavu
