#include <opencv2/opencv.hpp>
#include <opencv2/opencv_modules.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>
#include "opencv2/imgcodecs.hpp"
#include <filesystem>

cv::Mat imgL = cv::imread("right.png"); // path to left image is "../im0.png"
cv::Mat imgR = cv::imread("left.png"); // path to left image 

cv::Mat imgL_gray;
cv::Mat imgR_gray;
 
// Setting parameters for StereoSGBM algorithm
int minDisparity = 2;
int numDisparities = 80;
int blockSize = 17;
int disp12MaxDiff = 19;
int P1 = 0;
int P2 = 395;


// // cv::Ptr<cv::StereoSGBM> stereo = cv::StereoSGBM::create(minDisparity,numDisparities,blockSize,
// // disp12MaxDiff,uniquenessRatio,speckleWindowSize,speckleRange);

int main()
{ 

  cv::Ptr<cv::StereoSGBM> stereo = cv::StereoSGBM::create(minDisparity,numDisparities,blockSize, P1,P2,
disp12MaxDiff);

  // Initialize variables to store the maps for stereo rectification
  cv::Mat Left_Stereo_Map_x, Left_Stereo_Map_y, Right_Stereo_Map_x, Right_Stereo_Map_y;

  // Reading the mapping values for stereo image rectification
  cv::FileStorage cv_file2 = cv::FileStorage("rectify_map_imx219_160deg_1080p_new.yaml", cv::FileStorage::READ);
  cv_file2["map_l_1"] >> Left_Stereo_Map_x;
  cv_file2["map_l_2"] >> Left_Stereo_Map_y;
  cv_file2["map_r_1"] >> Right_Stereo_Map_x;
  cv_file2["map_r_2"] >> Right_Stereo_Map_y;
  cv_file2.release();
 

cv::Mat disp, disparity;
 
    // Converting images to grayscale
    cvtColor(imgL, imgL_gray, cv::COLOR_BGR2GRAY);
    cvtColor(imgR, imgR_gray, cv::COLOR_BGR2GRAY);
 
//     // Initialize matrix for rectified stereo images
    cv::Mat Left_nice, Right_nice;
 
    // Applying stereo image rectification on the left image
    cv::remap(imgL_gray,
              Left_nice,
              Left_Stereo_Map_x,
              Left_Stereo_Map_y,
              cv::INTER_LANCZOS4,
              cv::BORDER_CONSTANT,
              cv::Scalar());
 
    // Applying stereo image rectification on the right image
    cv::remap(imgR_gray,
              Right_nice,
              Right_Stereo_Map_x,
              Right_Stereo_Map_y,
              cv::INTER_LANCZOS4,
              cv::BORDER_CONSTANT,
              0);
 
    // Calculating disparith using the StereoBM algorithm
    stereo->compute(Left_nice,Right_nice,disp);
 
    // NOTE: Code returns a 16bit signed single channel image,
    // CV_16S containing a disparity map scaled by 16. Hence it 
    // is essential to convert it to CV_32F and scale it down 16 times.
 
    // Converting disparity values to CV_32F from CV_16S
    disp.convertTo(disparity,CV_32F, 1.0);
 
    // Scaling down the disparity values and normalizing them 
    disparity = (disparity/16.0f - (float)minDisparity)/((float)numDisparities);

    // // Normalizing the disparity map for better visualisation 
    // cv::normalize(disparity, disparity, 0, 255, cv::NORM_MINMAX, CV_8UC1);
 
    // Displaying the disparity map
    cv::imwrite("disparity.png",disparity);

  // std::cout << "Текущая рабочая директория: " << std::filesystem::current_path() << std::endl;
  return 0;
}