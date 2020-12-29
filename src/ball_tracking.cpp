#include <ball_tracking/ball_tracking.hpp>
#include <ball_tracking/tracking.h>// the tracking_gui message !

#include <sensor_msgs/image_encodings.h>

BallTracking::BallTracking () :
    image_topic_("/kinect2/sd/image_color_rect"),
    depth_topic_("/kinect2/sd/image_depth")
{
    // HSV inRange variables
    nh_.param("/five_axis_robot/hsv_variables/h_upper", hu_, 84);
    nh_.param("/five_axis_robot/hsv_variables/h_lower", hl_, 41);
    nh_.param("/five_axis_robot/hsv_variables/s_upper", su_, 376);
    nh_.param("/five_axis_robot/hsv_variables/s_lower", sl_, 90);
    nh_.param("/five_axis_robot/hsv_variables/v_upper", vu_, 132);
    nh_.param("/five_axis_robot/hsv_variables/v_lower", vl_, 70);

    // Minimum distance between circle centres
    nh_.param("/five_axis_robot/hough_variables/minimum_distance", hough_min_dist_, 2);
    // Upper threshold of the Canny edge function
    nh_.param("/five_axis_robot/hough_variables/param1", hough_param_1_, 231);
    // Centre detection threshold
    nh_.param("/five_axis_robot/hough_variables/hough_param_2", hough_param_2_, 13);

	image_sub_ = nh_.subscribe (image_topic_, 1, &BallTracking::image_cb, this);
	depth_sub_ = nh_.subscribe (depth_topic_, 1, &BallTracking::depth_cb, this);

    image_pub_ = nh_.advertise<sensor_msgs::Image> ("five_axis_image", 5);
    depth_pub_ = nh_.advertise<sensor_msgs::Image> ("five_axis_depth_image", 5);

    ball_target_pub_ = nh_.advertise<geometry_msgs::PointStamped> ("ball_target_point", 30);

    tracking_params_sub_ = nh_.subscribe("tracking_params", 1, &BallTracking::tracking_params_cb, this);

    tfListener_ptr_ = std::make_shared<tf2_ros::TransformListener>(tfBuffer_);
}

void BallTracking::image_cb (const sensor_msgs::Image& bgrImage)
{
    track_ball(bgrImage);
}

void BallTracking::depth_cb (const sensor_msgs::Image& depthImage)
{
    add_ball_circles(depthImage);
}

void BallTracking::tracking_params_cb(ball_tracking::tracking params)
{
    hu_ = params.hue_upper;
    hl_ = params.hue_lower;
    su_ = params.saturation_upper;
    sl_ = params.saturation_lower;
    vu_ = params.value_upper;
    vl_ = params.value_lower;
    hough_min_dist_ = params.hough_min_dist;
    hough_param_1_ = params.hough_param_1;
    hough_param_2_ = params.hough_param_2;

    show_hsv_window_ = params.show_hsv_window;
    show_hough_depth_image_ = params.show_hough_depth_image;
}

void BallTracking::track_ball(const sensor_msgs::Image& msg)
{
    std::vector<cv::Vec3f> VectorCirc;
    try
    {
        cv_bgr_ptr_ = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    Mat input_image = cv_bgr_ptr_->image;

    cv::flip(input_image, input_image, 1);

    Mat  hsv_image;

    cv::cvtColor(input_image, input_image, cv::COLOR_BGR2HSV);
    cv::inRange(input_image, cv::Scalar(hl_, sl_, vl_), cv::Scalar(hu_, su_, vu_), hsv_image);
    cv::GaussianBlur(hsv_image, hsv_image, cv::Size(9, 9), 1.5);
    cv::cvtColor(input_image, input_image, cv::COLOR_HSV2BGR);

    image_helper_.showHSVWindow(hsv_image, show_hsv_window_);

    HoughCircles(hsv_image, circles_, cv::HOUGH_GRADIENT, 1,
                 hsv_image.rows / hough_min_dist_,
                 hough_param_1_,
                 hough_param_2_, 1, 30);
}

void BallTracking::add_ball_circles(const sensor_msgs::Image& depthImage)
{
    if (cv_bgr_ptr_ == static_cast<boost::shared_ptr<cv_bridge::CvImage>>(nullptr))
        return;

    cv_bridge::CvImagePtr cv_depth_ptr;
    try
    {
        cv_depth_ptr = cv_bridge::toCvCopy(depthImage, enc::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    Mat depth_image = cv_depth_ptr->image;

    cv::flip(depth_image, depth_image, 1);

    cv::Mat mono_image(depth_image.size(), CV_8UC3);
    cv::cvtColor(depth_image, mono_image, CV_GRAY2RGB);

    std::for_each(circles_.cbegin(), circles_.cend(),[this, cv_depth_ptr, depth_image, mono_image](Vec3f hough_circle) {

        Point centre_px(cvRound(hough_circle[0]), cvRound(hough_circle[1]));
        Point depth_centre_px(centre_px);

        float z = cv_depth_ptr->image.at<float>(depth_centre_px);
        float x = z * image_helper_.fov_depth_width_ * (centre_px.x - cv_depth_ptr->image.size().width / 2.0) / cv_depth_ptr->image.size().width;
        float y = -(z * image_helper_.fov_depth_height_ * (centre_px.y - cv_depth_ptr->image.size().height / 2.0) / cv_depth_ptr->image.size().height); 

        Point3f centre(x*0.001, y*0.001, z*0.001); // cartesian in metres

        if (z > 500 && z < 1000)
        {
            Mat input_image = cv_bgr_ptr_->image;

            int radius = static_cast<int>(hough_circle[2]);

            cv::circle(input_image, centre_px, 3, Scalar(0), -1);
            cv::circle(input_image, centre_px, radius, Scalar(0, 0, 255), 2);

            std::ostringstream stringStream;
            stringStream.precision(3);
            stringStream.setf( std::ios::fixed, std:: ios::floatfield );
            stringStream << "(" << centre.x << ", " << centre.y << ", " << centre.z << ")" ;

            const int fontFace = FONT_HERSHEY_PLAIN;
            const double fontScale = 2;
            const int fontThickness = 2;
            int baseLine = 2;

            Size textSize = getTextSize(stringStream.str(), fontFace, fontScale, fontThickness, &baseLine);

            putText(input_image,
                     stringStream.str(),
                     Point(centre_px.x-(textSize.width/2), centre_px.y-radius-(baseLine)),
                     fontFace,
                     fontScale,
                     Scalar(0, 0, 255), // Colour
                     fontThickness);

            cv::circle(depth_image, depth_centre_px, 50, Scalar(0, 0, 0), 2);
            cv::circle(mono_image, depth_centre_px, radius, Scalar(0, 0, 0), 2);

            publishBallPosition(centre);
         }
     });

    image_helper_.showHoughDepthWindow(mono_image, show_hough_depth_image_);
    depth_pub_.publish(cv_depth_ptr->toImageMsg());
    image_pub_.publish(cv_bgr_ptr_->toImageMsg());
}

void BallTracking::publishBallPosition(const Point3f& centre)
{
    geometry_msgs::PointStamped target_point_msg;
             target_point_msg.header.frame_id = "kinect2_ir_optical_frame";
             target_point_msg.point.x = centre.x;
             target_point_msg.point.y = centre.y;
             target_point_msg.point.z = centre.z;

    ball_target_pub_.publish(target_point_msg);
}

int
main (int argc, char **argv)
{
  ros::init (argc, argv, "ball_tracking_node");
  BallTracking fat; //this loads up the node
  ros::spin (); //where she stops nobody knows
  return 0;
}
