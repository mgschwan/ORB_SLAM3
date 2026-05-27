#ifndef COLOR_UTILS_H
#define COLOR_UTILS_H

#include <opencv2/core/core.hpp>
#include <algorithm>

namespace ORB_SLAM3
{

// Average BGR color over a 5x5 kernel centered at pt, clamped to image bounds.
// Returns {0,0,0} when img is empty. Handles 1-, 3-, and 4-channel images;
// single-channel values are replicated across all three output channels.
inline cv::Vec3b sampleBGR5x5(const cv::Mat& img, const cv::Point2f& pt)
{
    if (img.empty()) return {0, 0, 0};
    const int cx = (int)pt.x, cy = (int)pt.y;
    int b = 0, g = 0, r = 0, n = 0;
    for (int dy = -2; dy <= 2; ++dy) {
        for (int dx = -2; dx <= 2; ++dx) {
            int x = std::max(0, std::min(cx + dx, img.cols - 1));
            int y = std::max(0, std::min(cy + dy, img.rows - 1));
            if (img.channels() == 3) {
                auto v = img.at<cv::Vec3b>(y, x);
                b += v[0]; g += v[1]; r += v[2];
            } else if (img.channels() == 4) {
                auto v = img.at<cv::Vec4b>(y, x);
                b += v[0]; g += v[1]; r += v[2];
            } else {
                int gv = img.at<uchar>(y, x);
                b += gv; g += gv; r += gv;
            }
            ++n;
        }
    }
    return {(uchar)(b/n), (uchar)(g/n), (uchar)(r/n)};
}

inline cv::Vec3b sampleBGR(const cv::Mat& img, const cv::Point2f& pt)
{
    if (img.empty()) return {0, 0, 0};
    const int x = std::max(0, std::min((int)pt.x, img.cols - 1));
    const int y = std::max(0, std::min((int)pt.y, img.rows - 1));
    if (img.channels() == 3) {
        return img.at<cv::Vec3b>(y, x);
    } else if (img.channels() == 4) {
        auto v = img.at<cv::Vec4b>(y, x);
        return {v[0], v[1], v[2]};
    } else {
        int gv = img.at<uchar>(y, x);
        return {gv, gv, gv};
    }
}


} // namespace ORB_SLAM3

#endif // COLOR_UTILS_H
