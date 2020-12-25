#ifndef IMAGEHELPER_H
#define IMAGEHELPER_H

#include <opencv2/imgproc.hpp>

using namespace cv;

class ImageHelper
{
public:
    ImageHelper();

    void showHoughDepthWindow(Mat rgb_image, bool show_hough_depth_image);
    void showHSVWindow(Mat hsv_image, bool show_hsv_image);

    // Kinect v2 FOV
    const float fov_rgb_width_ = 1.4678f;
    const float fov_rgb_height_ = 0.9389f;
    const float fov_depth_width_ = 1.2322f;
    const float fov_depth_height_ = 1.0471f;

private:
    bool hsv_window_open_ = false;
    bool hough_depth_window_open_ = false;
};

#endif // IMAGEHELPER_H
