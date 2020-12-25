#include <ball_tracking/imagehelper.h>
#include <opencv2/highgui/highgui.hpp>

ImageHelper::ImageHelper()
{
}

void ImageHelper::showHoughDepthWindow(Mat rgb_image, bool show_hough_depth_image)
{
    if (show_hough_depth_image)
    {
        if (hough_depth_window_open_ == false)
        {
            namedWindow("hough_depth", WINDOW_NORMAL);
            hough_depth_window_open_ = true;
        }

        cv::imshow("hough_depth", rgb_image);
        cv::waitKey(3);
    }
    else if (hough_depth_window_open_)
    {
        cv::destroyWindow("hough_depth");
        hough_depth_window_open_ = false;
    }
}

void ImageHelper::showHSVWindow(Mat hsv_image, bool show_hsv_image)
{
    if (show_hsv_image)
    {
        if (hsv_window_open_ == false)
        {
            namedWindow("hsv", WINDOW_NORMAL);
            hsv_window_open_ = true;
        }

        cv::imshow("hsv", hsv_image);
        cv::waitKey(3);
    }
    else if (hsv_window_open_)
    {
        cv::destroyWindow("hsv");
        hsv_window_open_ = false;
    }
}
