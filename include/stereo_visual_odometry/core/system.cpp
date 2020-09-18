#include "system.h"

namespace DVO
{
    system::system(const std::string &settings_file)
    {
        cv::namedWindow("window", cv::WINDOW_AUTOSIZE);
        std::cout << "settings file " << std::endl;
    }
    void system::StartVO(const cv::Mat &left_image, const cv::Mat &right_image, const double &time_stamp)
    {
        DVO::tracking *tracker;
        DVO::tracking left_tracking(tracker);
        DVO::tracking right_tracking(tracker);

        //create keypoints and descriptors 
        std::vector<cv::KeyPoint> left_keypoints, right_keypoints;
        cv::Mat left_descriptors, right_descriptors;

        //TODO: create a thread and left and right feature detection have to be perfomed by thread
        left_tracking.DetectAKAZEFeatures(left_image, left_keypoints, left_descriptors);
        right_tracking.DetectAKAZEFeatures(right_image, right_keypoints, right_descriptors);

        //Feature matching:

        
    }
    void system::ShutdownVO()
    {
        std::cout << "system shutdown" << std::endl;
    }
}