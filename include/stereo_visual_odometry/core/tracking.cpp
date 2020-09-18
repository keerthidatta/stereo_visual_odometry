#include "tracking.h"

namespace DVO{
void tracking::DetectAKAZEFeatures(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors)
{
    /*auto detector = cv::SiftFeatureDetector::create(1000);
    detector->detect(image, keypoints);

    auto extractor = cv::SiftDescriptorExtractor::create();
    extractor->compute(image, keypoints, descriptors);*/

    auto detector = cv::AKAZE::create();
    auto extractor = cv::AKAZE::create();
    

    detector->detect(image, keypoints);
    extractor->compute(image, keypoints, descriptors);
    std::cout << "Feature points: " << keypoints.size() << std::endl;

    /*
    cv::Mat out;
    cv::drawKeypoints(image, keypoints, out, cv::Scalar(255, 0, 0));
    cv::imshow("window", out);
    cv::waitKey(10);
    */

}

void tracking::MatchFeatures(cv::Mat& left_descriptors, cv::Mat& right_descriptors)
{
    cv::BFMatcher matcher(cv::NORM_HAMMING);

    std::vector<cv::DMatch> ratio_matches = MatchRatioTest(matcher, left_descriptors, right_descriptors);
    std::vector<cv::DMatch> reciprocal_matches = MatchRatioTest(matcher, right_descriptors, left_descriptors);

    //Brute Force matcher

    std::vector<cv::DMatch> merged_matches;
    for(const cv::DMatch& reciprocal_match : reciprocal_matches)
    {
        bool match_found = false;
        for(const cv::DMatch& ratio_match : ratio_matches)
        {
            if(reciprocal_match.queryIdx == ratio_match.trainIdx and reciprocal_match.trainIdx == ratio_match.queryIdx)
            {
                merged_matches.push_back(ratio_match);
                match_found = true;
                break;
            }
        }
        if(match_found)
            continue;
    }

}

std::vector<cv::DMatch> tracking::MatchRatioTest(const cv::DescriptorMatcher& matcher, const cv::Mat& descriptors1, const cv::Mat& descriptors2)
{
    std::vector<std::vector<cv::DMatch>> knn_match;
    matcher.knnMatch(descriptors1, descriptors2, knn_match, 2);

    //ration test
    std::vector<cv::DMatch> ratio_matches;
    for (size_t i = 0; i < knn_match.size(); i++)
    {
        cv::DMatch first = knn_match[i][0];
        float distance1 = knn_match[i][0].distance;
        float distance2 = knn_match[i][1].distance;
        if(distance1 < distance2)
        {
            ratio_matches.push_back(first);
        }
    }
    return ratio_matches;
}
}
