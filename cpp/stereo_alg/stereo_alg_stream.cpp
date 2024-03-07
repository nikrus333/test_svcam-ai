#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>
#include "opencv2/imgcodecs.hpp"
#include <filesystem>

using namespace std;

cv::Mat imgL = cv::imread("right.png"); // path to left image is "../im0.png"
cv::Mat imgR = cv::imread("left.png"); // path to left image 

cv::Mat imgL_gray;
cv::Mat imgR_gray;
 
// Setting parameters for StereoSGBM algorithm
int minDisparity = 2;
int numDisparities = 80;
int blockSize = 17;
int disp12MaxDiff = 19;
int uniquenessRatio = 10;
int speckleWindowSize = 10;
int speckleRange = 8;
 
int main()
{  
  std::string left_cam_pipeline = "udpsrc port=1234 ! application/x-rtp,payload=26 ! rtpjpegdepay ! jpegdec ! appsink";
  std::string right_cam_pipeline = "udpsrc port=1235 ! application/x-rtp,payload=26 ! rtpjpegdepay ! jpegdec !   appsink";

  cv::VideoCapture cap_left(left_cam_pipeline, cv::CAP_GSTREAMER);
  cv::VideoCapture cap_right(right_cam_pipeline, cv::CAP_GSTREAMER);
 
  // Check if left camera is attached

  if (!cap_left.isOpened())
  {
    std::cout << "Could not open camera with index : " << left_cam_pipeline << std::endl;
    return -1;
  }

  if (!cap_right.isOpened())
  {
    std::cout << "Could not open camera with index : " << right_cam_pipeline << std::endl;
    return -1;
  }

   cv::VideoWriter writer_left, writer_right, writer_disp;

    // Получаем ширину и высоту кадра с камеры
   int frame_width = static_cast<int>(cap_left.get(cv::CAP_PROP_FRAME_WIDTH));
   int frame_height = static_cast<int>(cap_left.get(cv::CAP_PROP_FRAME_HEIGHT));   
       // Формат, кодек, частота кадров и размер задаются здесь
   writer_right.open("output_left. mp4", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 10, cv::Size(frame_width, frame_height), true);   
   writer_left.open("output_right. mp4", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 10, cv::Size(frame_width, frame_height), true);  
   writer_disp.open("output_disp. mp4", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 10, cv::Size(frame_width, frame_height), true);    
   // Проверяем, удалось ли открыть файл для записи
   if (!writer_left.isOpened() || !writer_right.isOpened()) {
       std::cerr << "Ошибка: Невозможно открыть файл для записи" << std::endl;
       return -1;
   }

  // Initialize variables to store the maps for stereo rectification
  cv::Mat Left_Stereo_Map_x, Left_Stereo_Map_y, Right_Stereo_Map_x, Right_Stereo_Map_y, disp, disparity, frame_left, frame_right;

  // Reading the mapping values for stereo image rectification
  cv::FileStorage cv_file2 = cv::FileStorage("rectify_map_imx219_160deg_1080p_new.yaml", cv::FileStorage::READ);
  cv_file2["map_l_1"] >> Left_Stereo_Map_x;
  cv_file2["map_l_2"] >> Left_Stereo_Map_y;
  cv_file2["map_r_1"] >> Right_Stereo_Map_x;
  cv_file2["map_r_2"] >> Right_Stereo_Map_y;
  cv_file2.release();

  cv::Ptr<cv::StereoSGBM> stereo = cv::StereoSGBM::create(minDisparity,numDisparities,blockSize,
disp12MaxDiff,uniquenessRatio,speckleWindowSize,speckleRange);

    // Converting images to grayscale
    cvtColor(imgL, imgL_gray, cv::COLOR_BGR2GRAY);
    cvtColor(imgR, imgR_gray, cv::COLOR_BGR2GRAY);
 
//     // Initialize matrix for rectified stereo images
    cv::Mat Left_nice, Right_nice;
 
    // Calculating disparith using the StereoBM algorithm
    stereo->compute(Left_nice,Right_nice,disp);
 
    // NOTE: Code returns a 16bit signed single channel image,
    // CV_16S containing a disparity map scaled by 16. Hence it 
    // is essential to convert it to CV_32F and scale it down 16 times.
 
    // Converting disparity values to CV_32F from CV_16S
    disp.convertTo(disparity,CV_32F, 1.0);
 
    // Scaling down the disparity values and normalizing them 
    disparity = (disparity/16.0f - (float)minDisparity)/((float)numDisparities);

    // Normalizing the disparity map for better visualisation 
    cv::normalize(disparity, disparity, 0, 255, cv::NORM_MINMAX, CV_8UC1);
 
    // Displaying the disparity map
    cv::imwrite("disparity.png",disparity);

  while (true) {
        // Захват кадра из видеопотока
        cap_left >> frame_left;
        cap_right >> frame_right;

        // Проверка на окончание видеопотока
        if (frame_left.empty()) {
            break;
        }
        
        cv::cvtColor (frame_left, frame_left_G, cv::COLOR_RGB2GRAY)
        cv::cvtColor (frame_right,frame_right_G, cv::COLOR_RGB2GRAY)

        // Applying stereo image rectification on the left image
        cv::remap(frame_left_G,
                Left_nice,
                Left_Stereo_Map_x,
                Left_Stereo_Map_y,
                cv::INTER_LANCZOS4,
                cv::BORDER_CONSTANT,
                cv::Scalar());
    
        // Applying stereo image rectification on the right image
        cv::remap(frame_right_G,
                Right_nice,
                Right_Stereo_Map_x,
                Right_Stereo_Map_y,
                cv::INTER_LANCZOS4,
                cv::BORDER_CONSTANT,
                0);

        stereo->compute(Left_nice, Right_nice, disp);
        disp.convertTo(disparity, CV_32F, 1.0);

        // Нормализация карты диспаратности для отображения
        cv::normalize(disparity, disparity, 0, 255, cv::NORM_MINMAX, CV_8UC1);

        // Отображение кадра
         // Записываем кадр в файл
        writer_left.write(frame_left);
        writer_right.write(frame_right);
        writer_disp.write(disparity)

            // Выходим из цикла, если нажата клавиша 'q'
        char c = (char)cv::waitKey(25);
        if (c == 'q' || c == 27) {
            break;
        }
    }

        // Освобождаем ресурсы
  cap_left.release();
  cap_right.release();
  writer_left.release();
  writer_right.release();
  cv::destroyAllWindows();

  return 0;


}