/*
Copyright (c) Helio Perroni Filho <xperroni@gmail.com>

This file is part of Clarus.

Clarus is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

Clarus is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Clarus. If not, see <http://www.gnu.org/licenses/>.
*/

#include <clarus/vision/colors.hpp>

#include <clarus/core/list.hpp>
using clarus::List;

#include <clarus/vision/filters.hpp> // For CHANNEL_WISE macro

List<cv::Mat> colors::BGRL(const cv::Mat &bgr) {
    List<cv::Mat> bgrl = channels(bgr);
    bgrl.append(lightness(bgr));
    return bgrl;

/*
    int rows = bgr.rows;
    int cols = bgr.cols;

    int rows2 = rows / 2;
    int cols2 = cols / 2;

    List<cv::Mat> mosaic;
    mosaic.append(cv::Mat(rows2, cols2, CV_8U)); // Blue
    mosaic.append(cv::Mat(rows2, cols2, CV_8U)); // Green
    mosaic.append(cv::Mat(rows2, cols2, CV_8U)); // Red
    mosaic.append(cv::Mat(rows2, cols2, CV_8U)); // Luminance

    for (int i = 0; i < rows; i++) {
        int r = 1 - (i % 2);
        int i2 = i / 2;
        if (i2 >= rows2) {
            break;
        }

        const cv::Vec3b *row = bgr.ptr<cv::Vec3b>(i);
        uchar *a = mosaic[0 + 2 * r].ptr<uchar>(i2);
        uchar *b = mosaic[1 + 2 * r].ptr<uchar>(i2);
        uchar *m = a + cols2;
        uchar *n = b + cols2;

        if (r == 0) {
            uchar* c[] = {a, b};
            uchar* z[] = {m, n};
            for (int j = 0; j < cols; j++, row++) {
                int k = j % 2;
                if (c[k] < z[k]) {
                    *(c[k]) = (*row)[k];
                    c[k]++;
                }
            }
        }
        else {
            for (int j = 0; j < cols; j++, row++) {
                if (j % 2 != 0) {
                    if (a < m) {
                        *a = (*row)[2];
                        a++;
                    }
                }
                else {
                    if (b < n) {
                        // See: http://stackoverflow.com/a/596241/476920
                        const cv::Vec3b &pixel = *row;
                        uchar B = pixel[0];
                        uchar G = pixel[1];
                        uchar R = pixel[2];
                        *b = (B + G + G + G + G + R + R + R) >> 3;
                        b++;
                    }
                }
            }
        }
    }

    return mosaic;
*/
}

List<cv::Mat> colors::channels(const cv::Mat &image) {
    List<cv::Mat> projections;
    cv::split(image, *projections);
    return projections;
}

List<cv::Mat> colors::channels(const cv::Mat &image, int space) {
    return channels(convert(image, space));
}

cv::Mat colors::merge(const List<cv::Mat> &channels) {
    cv::Mat merged;
    cv::merge(*channels, merged);
    return merged;
}

cv::Mat colors::merge(const cv::Mat &c1, const cv::Mat &c2, const cv::Mat &c3) {
    List<cv::Mat> channels;
    channels.append(c1);
    channels.append(c2);
    channels.append(c3);
    return merge(channels);
}

cv::Mat colors::colormap(const cv::Mat &image, int cm) {
    cv::Mat colored;
    cv::applyColorMap(image, colored, cm);
    return colored;
}

cv::Mat colors::convert(const cv::Mat &image, int space) {
    cv::Mat out;
    cv::cvtColor(image, out, space);
    return out;
}

cv::Mat colors::discrete(const cv::Mat &data, bool normalize) {
    CHANNEL_WISE(discrete, data, normalize);

    cv::Mat values;
    if (normalize) {
        cv::normalize(data, values, 0, 255, CV_MINMAX);
    }
    else {
        values = data;
    }

    cv::Mat pixels;
    values.convertTo(pixels, CV_8U);

    return pixels;
}

cv::Mat colors::grayscale(const cv::Mat &bgr) {
    if (bgr.channels() == 1) {
        return bgr;
    }

    cv::Mat grays;
    cv::cvtColor(bgr, grays, CV_BGR2GRAY);
    return grays;
}

List<cv::Mat> colors::HLS(const cv::Mat &bgr) {
    cv::Mat hls;
    cv::cvtColor(bgr, hls, CV_BGR2HLS);
    List<cv::Mat> channels;
    cv::split(hls, *channels);
    return channels;
}

cv::Mat colors::hue(const cv::Mat &bgr) {
    List<cv::Mat> hls = HLS(bgr);
    return hls[0];
}

cv::Mat colors::lightness(const cv::Mat &bgr) {
    List<cv::Mat> hls = HLS(bgr);
    return hls[1];
}

cv::Mat colors::saturation(const cv::Mat &bgr) {
    List<cv::Mat> hls = HLS(bgr);
    return hls[2];
}
