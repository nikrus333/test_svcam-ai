import numpy as np 
import cv2
 
# # Check for left and right camera IDs
# # These values can change depending on the system
# CamL_id = 2 # Camera ID for left camera
# CamR_id = 0 # Camera ID for right camera
 
# CamL= cv2.VideoCapture(CamL_id)
# CamR= cv2.VideoCapture(CamR_id)

# Загрузка изображений со стереокамер
imgL = cv2.imread('/home/saharo4ek/Diplom/left_cam_img.png')
imgR = cv2.imread('/home/saharo4ek/Diplom/right_cam_img.png')

 
# Reading the mapping values for stereo image rectification
cv_file = cv2.FileStorage("/home/saharo4ek/Diplom/rectify_map_imx219_160deg_1080p_new.yaml", cv2.FILE_STORAGE_READ)
Left_Stereo_Map_x = cv_file.getNode("map_l_1").mat()
Left_Stereo_Map_y = cv_file.getNode("map_l_2").mat()
Right_Stereo_Map_x = cv_file.getNode("map_r_1").mat()
Right_Stereo_Map_y = cv_file.getNode("map_r_2").mat()
cv_file.release()
 
def nothing(x):
    pass
 
cv2.namedWindow('disp',cv2.WINDOW_NORMAL)
cv2.resizeWindow('disp',600,600)
 
cv2.createTrackbar('numDisparities', 'disp', 1, 17, nothing)
cv2.createTrackbar('blockSize', 'disp', 5, 50, nothing)
cv2.createTrackbar('P1', 'disp', 8, 200, nothing)
cv2.createTrackbar('P2', 'disp', 32, 400, nothing)
cv2.createTrackbar('disp12MaxDiff', 'disp', 5, 25, nothing)
cv2.createTrackbar('minDisparity', 'disp', 5, 25, nothing)
 
# Creating an object of StereoBM algorithm
stereo = cv2.StereoSGBM_create()
 
while True:
 
#   # Capturing and storing left and right camera images
#   retL, imgL= CamL.read()
#   retR, imgR= CamR.read()
   
#   # Proceed only if the frames have been captured
#   if retL and retR:
    imgR_gray = cv2.cvtColor(imgR,cv2.COLOR_BGR2GRAY)
    imgL_gray = cv2.cvtColor(imgL,cv2.COLOR_BGR2GRAY)
 
    # Applying stereo image rectification on the left image
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
 
     # Updating the parameters based on the trackbar positions
    numDisparities = cv2.getTrackbarPos('numDisparities', 'disp') * 16
    blockSize = cv2.getTrackbarPos('blockSize', 'disp') * 2 + 5
    P1 = cv2.getTrackbarPos('P1', 'disp')
    P2 = cv2.getTrackbarPos('P2', 'disp')
    disp12MaxDiff = cv2.getTrackbarPos('disp12MaxDiff', 'disp')
    minDisparity = cv2.getTrackbarPos('minDisparity', 'disp')

     
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
 
    # Scaling down the disparity values and normalizing them 
    disparity = (disparity/16.0 - minDisparity)/numDisparities
 
    # Displaying the disparity map
    cv2.imshow("disp",disparity)
 
    # Close window using esc key
    if cv2.waitKey(1) == 27:
      break
   
#   else:
#     CamL= cv2.VideoCapture(CamL_id)
#     CamR= cv2.VideoCapture(CamR_id)