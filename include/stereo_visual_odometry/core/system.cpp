#include "system.h"


namespace DVO
{
    system::system(const std::string &settings_file)
    {
        std::cout << "settings file " << std::endl;
    }
    void system::StartVO(const cv::Mat &left_image, const cv::Mat &right_image, const double &time_stamp)
    {
        
    }
    void system::ShutdownVO()
    {
        std::cout << "system shutdown" << std::endl;
    }
}