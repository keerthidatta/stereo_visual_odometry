#ifndef TRACKING_H
#define TRACKING_H
#include "system.h"
#include <opencv2/features2d/features2d.hpp>

const float MATCH_RATIO_THRESHOLD = 0.8f;
//bool VISUALIZE_FEATURE_MATCHES = true;
const float matchSurvivalRate = 0.5f;

namespace DVO
{
    class tracking
    {
    public:
        tracking(std::unique_ptr<tracking> tracker):tracker_(std::move(tracker)){}
        
        //feature detectors
        void DetectAKAZEFeatures(const cv::Mat& image, 
                                 std::vector<cv::KeyPoint> &keypoints, 
                                 cv::Mat& descriptors);
        
        //matching techniques
        void MatchFeatures(const cv::Mat &left_image, const cv::Mat &right_image,
            std::vector<cv::KeyPoint> &left_keypoints, 
                           std::vector<cv::KeyPoint> &right_keypoints,
                           cv::Mat& left_descriptors, 
                           cv::Mat& right_descriptors);

        std::vector<cv::DMatch> MatchRatioTest(const cv::DescriptorMatcher& matcher, 
                                               const cv::Mat& descriptors1,
                                               const cv::Mat& descriptors2);

    private:
        //tracking *tracker;
        std::unique_ptr<tracking> tracker_;
    };

}
#endif