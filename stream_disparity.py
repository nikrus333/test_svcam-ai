
import numpy as np
import os
import urllib.request
from PIL import Image
import cv2
import threading
from http.server import BaseHTTPRequestHandler,HTTPServer
from socketserver import ThreadingMixIn
from io import StringIO,BytesIO
import time


class CamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        
        if self.path.endswith('.mjpg'):
            self.send_response(200)
            self.send_header('Content-type','multipart/x-mixed-replace; boundary=--jpgboundary')
            self.end_headers()

            stereo = cv2.StereoSGBM_create()
            
            numDisparities = 5 * 16
            blockSize = 6 * 2+ 5
            P1 = 0
            P2 = 395
            disp12MaxDiff = 19 
            minDisparity = 2

            stereo.setNumDisparities(numDisparities)
            stereo.setBlockSize(blockSize)
            stereo.setP1(P1)
            stereo.setP2(P2)
            stereo.setDisp12MaxDiff(disp12MaxDiff)
            stereo.setMinDisparity(minDisparity)
            while True:
                try:
                    
                    rc_l, img_left = capture_left.read()
                    rc_r, img_right = capture_right.read()
                    imgL_gray=cv2.cvtColor(img_left, cv2.COLOR_BGR2GRAY)
                    imgR_gray=cv2.cvtColor(img_right, cv2.COLOR_BGR2GRAY)
                    Left_nice= cv2.remap(imgL_gray,
                                        Left_Stereo_Map_x,
                                        Left_Stereo_Map_y,
                                        cv2.INTER_LANCZOS4,
                                        cv2.BORDER_CONSTANT,
                                        0)
                    
                    Right_nice= cv2.remap(imgR_gray,
                                            Right_Stereo_Map_x,
                                            Right_Stereo_Map_y,
                                            cv2.INTER_LANCZOS4,
                                            cv2.BORDER_CONSTANT,
                                            0)
                    print(rc_l, rc_r)
                    # img_left = cv2.resize(img_left, (416, 416))
                    # img_right = cv2.resize(img_right, (416, 416))
                    #dst_left = cv2.undistort(img_left, camera_matrix_left, dist_coefs_left, None, camera_matrix_left)
                    #dst_right = cv2.undistort(img_right, camera_matrix_right, dist_coefs_right, None, camera_matrix_right)
                    vis = img_left
                    print(type(vis))
                    print(vis.shape)
                    print(vis)
                    disparity = stereo.compute(Left_nice, Right_nice)
                    disparity = disparity.astype(np.float32)
                    disparity = (disparity/16.0 - minDisparity)/numDisparities
                    print(type(disparity))
                    print(disparity.shape)
                    #vis=cv2.cvtColor(disparity, cv2.COLOR_GRAY2BGR)
                    #norm_image = cv2.normalize(disparity, None, alpha = 0, beta = 1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
                    #disparity = np.resize(disparity, (480, 640, 3))
                    #disparity = cv2.normalize(disparity, disparity, alpha=255,
                     #         beta=0, norm_type=cv2.NORM_MINMAX)
                    cv2.imshow("disp",disparity)
                    #cv2.imwrite('disp.ipg', disparity)
                    #disparity = cv2.imride('disp.ipg')
                    vis = disparity
                    print(type(vis))
                    print(vis.shape)
                    print(vis)
                    jpg = Image.fromarray(vis)
                    tmpFile = BytesIO()
                    jpg.save(tmpFile,'JPEG')
                    self.wfile.write("--jpgboundary".encode())
                    self.send_header('Content-type','image/jpeg')
                    self.send_header('Content-length',str(tmpFile.getbuffer().nbytes))
                    self.end_headers()
                    jpg.save(self.wfile,'JPEG')
                    time.sleep(0.05)
                except KeyboardInterrupt:
                    break
            return

        if self.path.endswith('.html'):
            self.send_response(200)
            self.send_header('Content-type','text/html')
            self.end_headers()
            self.wfile.write('<html><head></head><body>'.encode())
            self.wfile.write('<img src="http://192.168.42.241/cam.mjpg"/>'.encode())
            self.wfile.write('</body></html>'.encode())
            return

class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
	"""Handle requests in a separate thread."""

CAP_WIDTH = 640
CAP_HEIGHT = 480

if __name__ == "__main__":
    
    
#     CAPTURE_PIPE_LEFT = (
#     "libcamerasrc camera-name=/base/i2c@ff110000/ov4689@36 !"
#     f"video/x-raw,width={CAP_WIDTH},height={CAP_HEIGHT},format=YUY2 ! videoconvert ! appsink"
# )

    #right
    CAPTURE_PIPE_l = (
    "libcamerasrc camera-name=/base/i2c@ff110000/ov4689@36 !"
    f"video/x-raw,width={CAP_WIDTH},height={CAP_HEIGHT},format=YUY2 ! videoconvert ! appsink"
)
    CAPTURE_PIPE_r = (
    "libcamerasrc camera-name=/base/i2c@ff120000/ov4689@36 !"
    f"video/x-raw,width={CAP_WIDTH},height={CAP_HEIGHT},format=YUY2 ! videoconvert ! appsink"
)
    
    global capture_left
    global img_left
    global img_right
    global capture_right

    capture_left = cv2.VideoCapture(CAPTURE_PIPE_l, cv2.CAP_GSTREAMER)
    capture_right = cv2.VideoCapture(CAPTURE_PIPE_r, cv2.CAP_GSTREAMER)
    cv_file = cv2.FileStorage("calibration/rectify_map_imx219_160deg_1080p_new.yaml", cv2.FILE_STORAGE_READ)
    global Left_Stereo_Map_x 
    global Left_Stereo_Map_y 
    global Right_Stereo_Map_x 
    global Right_Stereo_Map_y 
    Left_Stereo_Map_x = cv_file.getNode("map_l_1").mat()
    Left_Stereo_Map_y = cv_file.getNode("map_l_2").mat()
    Right_Stereo_Map_x = cv_file.getNode("map_r_1").mat()
    Right_Stereo_Map_y = cv_file.getNode("map_r_2").mat()
    cv_file.release()

    try:
        while True:
            server = ThreadedHTTPServer(('192.168.42.241', 8087), CamHandler)
            print( "server started")
            server.serve_forever()
    except KeyboardInterrupt:
            print("here")
            capture_left.release()
            server.socket.close()


    