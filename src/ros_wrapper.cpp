#include <iostream>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

//Include core modules of visual odometry
#include "../include/stereo_visual_odometry/core/system.h"
#include "../include/stereo_visual_odometry/core/system.cpp"
#include "../include/stereo_visual_odometry/core/tracking.h"
#include "../include/stereo_visual_odometry/core/tracking.cpp"

std::string LEFT_IMAGE_TOPIC_DEFAULT = "/left/stereo_camera/image_raw"; 
std::string RIGHT_IMAGE_TOPIC_DEFAULT = "/right/stereo_camera/image_raw";

class ros_wrapper
{
    public:
        ros_wrapper(DVO::system* vo):vo_(vo){}
        void stereo_image_callback(const sensor_msgs::ImageConstPtr& left_image,const sensor_msgs::ImageConstPtr& right_image);
        DVO::system* vo_;
};

void ros_wrapper::stereo_image_callback(const sensor_msgs::ImageConstPtr& left_image,const sensor_msgs::ImageConstPtr& right_image)
{
    //copy ros image to cv pointer
    cv_bridge::CvImageConstPtr cv_ptr_left;
    try
    {
        cv_ptr_left = cv_bridge::toCvShare(left_image);
    }
    catch(cv_bridge::Exception &e)
    {
        ROS_ERROR("CV_BRIDGE EXCEPTION: %s", e.what());
        return;
    }
    cv_bridge::CvImageConstPtr cv_ptr_right;
    try
    {
        cv_ptr_right = cv_bridge::toCvShare(right_image);
    }
    catch(const cv::Exception& e)
    {
        ROS_ERROR("CV_BRIDGE EXCEPTION: %s", e.what());
        return;
    }
    vo_->StartVO(cv_ptr_left->image, cv_ptr_right->image, cv_ptr_left->header.stamp.toSec());
}

int main(int argc, char **argv)
{
    std::string calibration_path = "../config/calibration.yaml";
    ros::init(argc, argv, "stereo_visual_odometry");
    ros::start();
    ros::NodeHandle nh_;	

    nh_.getParam("LEFT_IMAGE_TOPIC", LEFT_IMAGE_TOPIC_DEFAULT);
    nh_.getParam("RIGHT_IMAGE_TOPIC", LEFT_IMAGE_TOPIC_DEFAULT);
    
    nh_.getParam("CALIBRATION_FILE_PATH",calibration_path);
    DVO::system system(calibration_path);
    ros_wrapper wrapper(&system);

    message_filters::Subscriber<sensor_msgs::Image> left_image(nh_, LEFT_IMAGE_TOPIC_DEFAULT, 1);
    message_filters::Subscriber<sensor_msgs::Image> right_image(nh_, RIGHT_IMAGE_TOPIC_DEFAULT, 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), left_image, right_image);
    sync.registerCallback(boost::bind(&ros_wrapper::stereo_image_callback,&wrapper, _1, _2));

    ros::spin();
    //ros::shutdown
}

