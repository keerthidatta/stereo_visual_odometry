#include "system.h"
#include <thread>
namespace DVO
{
    system::system(const std::string &settings_file)
    {
        cv::namedWindow("window", cv::WINDOW_AUTOSIZE);
        std::cout << "settings file " << std::endl;
    }
    void system::StartVO(const cv::Mat &left_image, const cv::Mat &right_image, const double &time_stamp)
    {
        std::unique_ptr<DVO::tracking> tracker;
        //DVO::tracking *tracker;
        DVO::tracking tracking(std::move(tracker));

        //create keypoints and descriptors 
        std::vector<cv::KeyPoint> left_keypoints, right_keypoints;
        cv::Mat left_descriptors, right_descriptors;

        //TODO: create a thread and left and right feature detection have to be perfomed by thread
        /*
        std::thread t1 = std::thread(&DVO::tracking::DetectAKAZEFeatures, left_image, left_keypoints, left_descriptors);
        std::thread t2 = std::thread(&DVO::tracking::DetectAKAZEFeatures, right_image, right_keypoints, right_descriptors);
        t1.join();
        t2.join();
        */

        tracking.DetectAKAZEFeatures(left_image, left_keypoints, left_descriptors);
        tracking.DetectAKAZEFeatures(right_image, right_keypoints, right_descriptors);

        //Feature matching:
        tracking.MatchFeatures(left_image, right_image, left_keypoints, right_keypoints, left_descriptors, right_descriptors);
        
    }
    void system::ShutdownVO()
    {
        std::cout << "system shutdown" << std::endl;
    }
}