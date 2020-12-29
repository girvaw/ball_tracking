#include <ros/ros.h>
#include <ros/console.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <ball_tracking/tracking.h>
#include <ball_tracking/imagehelper.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PointStamped.h>

#include <string>

using namespace cv;
namespace enc = sensor_msgs::image_encodings;

class BallTracking
{
public:
  BallTracking () ;
  void image_cb (const sensor_msgs::Image& bgrImage);
  void depth_cb (const sensor_msgs::Image& depthImage);
  void tracking_params_cb(ball_tracking::tracking params);

private:
  void track_ball(const sensor_msgs::Image& msg);
  void add_ball_circles(const sensor_msgs::Image& depthImage);
  void publishBallPosition(const Point3f& centre);

  private:
    ros::NodeHandle nh_;

  	std::string image_topic_; 
  	std::string depth_topic_; 
  	std::string joint_state_topic_; 

  	ros::Subscriber depth_sub_; 
  	ros::Subscriber image_sub_; 
    ros::Subscriber ball_target_sub_;

    ros::Publisher image_pub_; 
    ros::Publisher depth_pub_;
    ros::Publisher ball_target_pub_;

    ros::Subscriber tracking_params_sub_;

    std::shared_ptr<Point> ball_centre_;

    int hl_, sl_, vl_, hu_, su_, vu_;
    int hough_min_dist_, hough_param_1_, hough_param_2_;
    bool show_hsv_window_ = false;
    bool show_hough_depth_image_ = false;
    std::vector<Vec3f> circles_;

    cv_bridge::CvImagePtr cv_bgr_ptr_ = nullptr;

    ImageHelper image_helper_;

    tf2_ros::Buffer tfBuffer_;
    std::shared_ptr<tf2_ros::TransformListener> tfListener_ptr_;
};
