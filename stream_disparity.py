
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
            while True:
                try:
                    camera_matrix_left = np.array([[423.38047293,   0.        , 367.10500348],
                                                [  0.        , 726.30866529, 267.70218356],
                                                [  0.        ,   0.        ,   1.        ]])
                    camera_matrix_right = np.array([[851.79411711,   0.        , 409.01096518],
                                                [  0.        , 832.54482296, 208.88240441],
                                                [  0.        ,   0.        ,   1.        ]])
                    rc_l, img_left = capture_left.read()
                    rc_r, img_right = capture_right.read()
                    img_left=cv2.cvtColor(img_left, cv2.COLOR_BGR2GRAY)
                    img_right=cv2.cvtColor(img_right, cv2.COLOR_BGR2GRAY)
                    print(rc_l, rc_r)
                    # img_left = cv2.resize(img_left, (416, 416))
                    # img_right = cv2.resize(img_right, (416, 416))
                    #dst_left = cv2.undistort(img_left, camera_matrix_left, dist_coefs_left, None, camera_matrix_left)
                    #dst_right = cv2.undistort(img_right, camera_matrix_right, dist_coefs_right, None, camera_matrix_right)
                    stereo = cv2.StereoBM_create(numDisparities=16, blockSize=15)
                    disparity = stereo.compute(img_left, img_right)
                    disparity.astype(np.float32)
                    vis = disparity
                    print(vis.shape)
                    
                    vis = cv2.cvtColor(vis, cv2.COLOR_BGR2GRAY)
                    if not rc_l:
                    	 continue
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

CAP_WIDTH = 480
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

    try:
        while True:
            server = ThreadedHTTPServer(('192.168.42.241', 8087), CamHandler)
            print( "server started")
            server.serve_forever()
    except KeyboardInterrupt:
            print("here")
            capture_left.release()
            server.socket.close()


    