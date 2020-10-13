#include <iostream>
#include<chrono>
#include "../core/system.h"

void load_image_left(cv::Mat &left_image, int frame, std::string images_sequence_path);
void load_image_right(cv::Mat &right_image, int frame, std::string images_sequence_path);

int main(int argc, char **argv)
{
    if(argc < 2)
    {
        std::cerr << "Usage: ./stereo_visual_odometry path_to_calibration" << std::endl;
        return 1;
    }

    // Camera calibration
    std::string calibrationpath = std::string(argv[1]);
    std::cout << "Calibration Filepath: " << calibrationpath << std::endl;

    cv::FileStorage fSettings(calibrationpath, cv::FileStorage::READ);
    std::string images_sequence_path = fSettings["PATH_TO_IMAGE_SEQUENCE"];

    DVO::system system(calibrationpath);
    cv::Mat left_image, right_image;
    int number_frames = 100;
    int init_frame = 48978;

    for (int frame=init_frame;  frame<init_frame+number_frames; frame++)
    {
        load_image_left(left_image, frame, images_sequence_path);
        load_image_right(right_image, frame, images_sequence_path);
        system.StartVO(left_image, right_image, 0.1);
    }
}

void load_image_left(cv::Mat &left_image, int frame, std::string images_sequence_path)
{
    char file[200];
    //mars color seq 22
    sprintf(file, "left_images/color-rectified-left-%06d.ppm", frame);
    std::string filename = images_sequence_path + std::string(file);
    left_image = cv::imread(filename, cv::IMREAD_COLOR);
}
void load_image_right(cv::Mat &right_image, int frame, std::string images_sequence_path)
{
    char file[200];
    //mars color seq 22
    sprintf(file, "right_images/color-rectified-right-%06d.ppm", frame);
    std::string filename = images_sequence_path + std::string(file);
    right_image = cv::imread(filename, cv::IMREAD_COLOR);
}