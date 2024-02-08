
import cv2
import numpy as np
import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 

def nothing():
    pass

if __name__ == '__main__':

    print(cv2.getBuildInformation())
    flag_manual = True
    pipeline = 'udpsrc port=1234 ! application/x-rtp,payload=26 ! rtpjpegdepay ! jpegdec !   appsink'
    cap_left = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
 
    
    pipeline2 = 'udpsrc port=1235 ! application/x-rtp,payload=26 ! rtpjpegdepay ! jpegdec !   appsink'
    cap_right = cv2.VideoCapture(pipeline2, cv2.CAP_GSTREAMER)
    cv_file = cv2.FileStorage("rectify_map_imx219_160deg_1080p_new.yaml", cv2.FILE_STORAGE_READ)
    Left_Stereo_Map_x = cv_file.getNode("map_l_1").mat()
    Left_Stereo_Map_y = cv_file.getNode("map_l_2").mat()
    Right_Stereo_Map_x = cv_file.getNode("map_r_1").mat()
    Right_Stereo_Map_y = cv_file.getNode("map_r_2").mat()
    cv_file.release()


    cv2.namedWindow('disp',cv2.WINDOW_NORMAL)
    cv2.resizeWindow('disp',600,600)
 
    cv2.createTrackbar('numDisparities', 'disp', 1, 17, nothing)
    cv2.createTrackbar('blockSize', 'disp', 5, 50, nothing)
    cv2.createTrackbar('P1', 'disp', 8, 200, nothing)
    cv2.createTrackbar('P2', 'disp', 32, 400, nothing)
    cv2.createTrackbar('disp12MaxDiff', 'disp', 5, 25, nothing)
    cv2.createTrackbar('minDisparity', 'disp', 5, 25, nothing)
 
    count = 0
    stereo = cv2.StereoSGBM_create()
    while True:
        count +=1
        ret_left, left_img = cap_left.read()
        ret_right, right_img = cap_right.read()
        #print(right_img.shape)
        imgR_gray = cv2.cvtColor(right_img, cv2.COLOR_RGB2GRAY)
        imgL_gray = cv2.cvtColor(left_img, cv2.COLOR_RGB2GRAY)
        Left_nice= cv2.remap(imgL_gray,
              Left_Stereo_Map_x,
              Left_Stereo_Map_y,
              cv2.INTER_LANCZOS4,
              cv2.BORDER_CONSTANT,
              0)
     
    # Applying stereo image rectification on the right image
        Right_nice= cv2.remap(imgR_gray,
              Right_Stereo_Map_x,
              Right_Stereo_Map_y,
              cv2.INTER_LANCZOS4,
              cv2.BORDER_CONSTANT,
              0)
        if flag_manual:
        
            numDisparities = cv2.getTrackbarPos('numDisparities', 'disp') * 16
            blockSize = cv2.getTrackbarPos('blockSize', 'disp') * 2 + 5
            P1 = cv2.getTrackbarPos('P1', 'disp')
            P2 = cv2.getTrackbarPos('P2', 'disp')
            disp12MaxDiff = cv2.getTrackbarPos('disp12MaxDiff', 'disp')
            minDisparity = cv2.getTrackbarPos('minDisparity', 'disp')
        else:
            numDisparities = 5 * 16
            blockSize = 6 * 2+ 5
            P1 = 0
            P2 = 395
            disp12MaxDiff = 19 
            minDisparity = 2
        
        # Setting the updated parameters before computing disparity map
        stereo.setNumDisparities(numDisparities)
        stereo.setBlockSize(blockSize)
        stereo.setP1(P1)
        stereo.setP2(P2)
        stereo.setDisp12MaxDiff(disp12MaxDiff)
        stereo.setMinDisparity(minDisparity)
    
        # Calculating disparity using the StereoBM algorithm
        disparity = stereo.compute(Left_nice,Right_nice)
        # NOTE: Code returns a 16bit signed single channel image,
        # CV_16S containing a disparity map scaled by 16. Hence it 
        # is essential to convert it to CV_32F and scale it down 16 times.
    
        # Converting to float32 
        disparity = disparity.astype(np.float32)
        print(disparity.shape)
    
        # Scaling down the disparity values and normalizing them 
        disparity = (disparity/16.0 - minDisparity)/numDisparities
    
        # Displaying the disparity map
        cv2.imshow("disp1",disparity)
        cv2.imshow("left_img",left_img)
        if cv2.waitKey(1) == 27:
            break
    
        # Close window using esc key
            
        