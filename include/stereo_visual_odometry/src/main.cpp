#include <iostream>
#include "../core/system.h"

int main(int argc, char **argv)
{
    std::cout << "out" << std::endl;
    if(argc < 3)
    {
        std::cerr << "Usage: ./stereo_visual_odometry path_to_sequence path_to_calibration" << std::endl;
        return 1;
    }

    // Sequence
    std::string filepath = std::string(argv[1]);
    std::cout << "Filepath: " << filepath << std::endl;

    // Camera calibration
    std::string calibrationpath = std::string(argv[2]);
    std::cout << "Calibration Filepath: " << calibrationpath << std::endl;

    cv::Mat left_image, right_image;

    DVO::system system(filepath);
}
