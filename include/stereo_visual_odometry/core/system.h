#ifndef SYSTEM_H
#define SYSTEM_H
#include <iostream>
#include <vector>
#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "tracking.h"

namespace DVO{

    class tracking;
    class system{
        public:
            system(const std::string &settings_file);

            //StartVO: Start visual odometry
            //Function takes left, right image and time stamp   
            void StartVO(const cv::Mat &left_image, const cv::Mat &right_image, const double &time_stamp);

            //Clears memory and shuts down system.
            void ShutdownVO();

    };
}
#endif // SYSTEM_H
