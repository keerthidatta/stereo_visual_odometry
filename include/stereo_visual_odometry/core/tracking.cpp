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
    //std::cout << "Feature points: " << keypoints.size() << std::endl;

    /*
    cv::Mat out;
    cv::drawKeypoints(image, keypoints, out, cv::Scalar(255, 0, 0));
    cv::imshow("window", out);
    cv::waitKey(10);
    */

}

void tracking::MatchFeatures(const cv::Mat &left_image, const cv::Mat &right_image, std::vector<cv::KeyPoint> &left_keypoints, 
                           std::vector<cv::KeyPoint> &right_keypoints, cv::Mat& left_descriptors, cv::Mat& right_descriptors)
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

    //Filter using epipolar constraint
    std::vector<uint8_t> inliers_mask(merged_matches.size());
    std::vector<cv::Point2f> img_left_points, img_right_points;
    for(const auto& match:merged_matches)
    {
        img_left_points.push_back(left_keypoints[match.queryIdx].pt);
        img_right_points.push_back(right_keypoints[match.trainIdx].pt);
    }
    cv::findFundamentalMat(img_left_points, img_right_points, inliers_mask);

    std::vector<cv::DMatch> final_matches;
    for(size_t m=0; m<merged_matches.size(); m++)
    {
        if(inliers_mask[m])
            final_matches.push_back(merged_matches[m]);
    }
    if((float) final_matches.size()/(float)ratio_matches.size() < matchSurvivalRate)
    {
        std::cout << "inliers count :" << (float) final_matches.size()/(float)ratio_matches.size() << std::endl;
        std::cout << "less inliers debug" << std::endl;
        
    }
    
    std::cout << "correct matches: " << final_matches.size() << std::endl;
    cv::Mat out;
    cv::drawMatches(left_image, left_keypoints,
                    right_image, right_keypoints,
                    final_matches, out, CV_RGB(255,0,0));
    cv::imshow("window", out);
    cv::waitKey(3);                
    

    /*


    //std::cout << "key1 size: " << left_keypoints.size() << "right: " << right_keypoints.size() <<std::endl;
    cv::Mat out;
    std::vector<cv::DMatch> raw_matches;
    matcher.match(left_descriptors, right_descriptors, raw_matches);
    std::vector<std::pair<std::string, std::vector<cv::DMatch>& > > showList{
                        {"Raw Match", raw_matches},
                        {"Ratio Test Filter", ratio_matches},
                        {"Reciprocal Filter", merged_matches},
                        {"Epipolar Filter", final_matches}
                    };

    std::cout << "size of showlist" << showList.size() << std::endl;
    for (size_t i = 0; i< showList.size(); i++) {
                        cv::drawMatches(left_image, left_keypoints,
                                    right_image, right_keypoints,
                                    showList[i].second, out, CV_RGB(255,0,0));
                        cv::putText(out, showList[i].first, cv::Point(10,50), cv::FONT_HERSHEY_COMPLEX, 2.0, CV_RGB(255,255,255), 2);
                        cv::putText(out, "# Matches: " + std::to_string(showList[i].second.size()), cv::Point(10,100), cv::FONT_HERSHEY_COMPLEX, 1.0, CV_RGB(255,255,255));
                        cv::imwrite("a_"+ std::to_string(i) + ".jpg", out);
                    }
    */

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
        if(distance1 < MATCH_RATIO_THRESHOLD * distance2)
        {
            ratio_matches.push_back(first);
        }
    }
    return ratio_matches;
}
}
