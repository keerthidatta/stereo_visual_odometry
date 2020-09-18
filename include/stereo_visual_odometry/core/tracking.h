#ifndef TRACKING_H
#define TRACKING_H
#include "system.h"
#include <opencv2/features2d/features2d.hpp>

const float MATCH_RATIO_THRESHOLD = 0.8f;
namespace DVO
{
    class tracking
    {
    public:
        tracking(tracking *tracker):tracker(tracker){}
        
        //feature detectors
        void DetectAKAZEFeatures(const cv::Mat& image, 
                                 std::vector<cv::KeyPoint> &keypoints, 
                                 cv::Mat& descriptors);
        
        //matching techniques
        void MatchFeatures(cv::Mat& left_descriptors, 
                           cv::Mat& right_descriptors);

        std::vector<cv::DMatch> MatchRatioTest(const cv::DescriptorMatcher& matcher, 
                                               const cv::Mat& descriptors1,
                                               const cv::Mat& descriptors2);

    private:
        tracking *tracker;
    };

}
#endif