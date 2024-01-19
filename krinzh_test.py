#!/usr/bin/env python3
from pickle import NONE
import numpy as np
import os
import urllib.request
from PIL import Image
import cv2
import threading
from http.server import BaseHTTPRequestHandler,HTTPServer
from socketserver import ThreadingMixIn
from io import StringIO,BytesIO
import datetime
import time

CAP_WIDTH = 768
CAP_HEIGHT = 432
current_time = None
start_time = None
video_duration = 30

left_cam_directory = ("/home/root/left_cam") 
right_cam_directory = ("/home/root/right_cam")

video_fps = 30.0
global video_writer_left 

class CamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        
        if self.path.endswith('.mjpg'):
            self.send_response(200)
            self.send_header('Content-type','multipart/x-mixed-replace; boundary=--jpgboundary')
            self.end_headers()
            start_time = time.time()
            current_time = datetime.datetime.now()  
            video_filename = current_time.strftime("%Y-%m-%d_%H-%M-%S") + '.avi'                      
            video_path_left = os.path.join(left_cam_directory, video_filename) 
            video_path_right = os.path.join(right_cam_directory, video_filename) 
            video_writer_left = cv2.VideoWriter(video_path_left, cv2.VideoWriter_fourcc(*'XVID'), video_fps, (CAP_WIDTH , CAP_HEIGHT))
            video_writer_right = cv2.VideoWriter(video_path_right, cv2.VideoWriter_fourcc(*'XVID'), video_fps, (CAP_WIDTH , CAP_HEIGHT))
            while True:
                try:   


                    elapsed_time = time.time() - start_time
    
                    current_time = datetime.datetime.now()                     
                    
                    # print(f"Video recorded: {video_filename}")
                    rc_l, img_left = capture_left.read()
                    rc_r, img_right = capture_right.read()
                    #print(rc_l, rc_r)
                    if not rc_l:
                        continue    
                    video_writer_left.write(img_left)
                    video_writer_right.write(img_right)
                    # count+=1
                    vis = np.concatenate((img_left, img_right), axis=1)
                    # print(vis.shape)
                    vis = cv2.cvtColor(vis, cv2.COLOR_BGR2RGB)
                    jpg = Image.fromarray(vis)
                    tmpFile = BytesIO()
                    jpg.save(tmpFile,'JPEG')
                    self.wfile.write("--jpgboundary".encode())
                    self.send_header('Content-type','image/jpeg')
                    self.send_header('Content-length',str(tmpFile.getbuffer().nbytes))
                    self.end_headers()
                    jpg.save(self.wfile,'JPEG')
                    time.sleep(0.05)
                    # if count > 100:
                    #   video_writer_left.release()
                        #   count=0

                    print(elapsed_time)
                    if elapsed_time >= video_duration:
                       
                        print("here")
                        
                        if  video_writer_left :
                             video_writer_left .release()
                             print("Video recorded.")
                             video_filename = current_time.strftime("%Y-%m-%d_%H-%M-%S") + '.avi'                      
                             video_path_left = os.path.join(left_cam_directory, video_filename) 
                             video_writer_left = cv2.VideoWriter(video_path_left, cv2.VideoWriter_fourcc(*'XVID'), video_fps, (CAP_WIDTH , CAP_HEIGHT))

                        if  video_writer_right :
                             video_writer_right.release()
                             print("Video recorded.")
                             video_filename = current_time.strftime("%Y-%m-%d_%H-%M-%S") + '.avi'                      
                             video_path_right = os.path.join(right_cam_directory, video_filename) 
                             video_writer_right = cv2.VideoWriter(video_path_right, cv2.VideoWriter_fourcc(*'XVID'), video_fps, (CAP_WIDTH , CAP_HEIGHT))

                        start_time = time.time()


                except KeyboardInterrupt:

                    if video_writer_left:
                         video_writer_left.release()
                    if video_writer_right:
                         video_writer_right.release()
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
          

if __name__ == "__main__":
    
    CAPTURE_PIPE_l = (
    "libcamerasrc camera-name=/base/i2c@ff110000/ov4689@36 !"
    f"video/x-raw,width={CAP_WIDTH},height={CAP_HEIGHT},format=YUY2 ! videoconvert ! appsink"
)
    CAPTURE_PIPE_r = (           
    "libcamerasrc camera-name=/base/i2c@ff120000/ov4689@36 !"
    f"video/x-raw,width={CAP_WIDTH},height={CAP_HEIGHT},format=YUY2 ! videoconvert ! appsink"
)

    global capture_left
    global capture_right


    #create directory if it doesn't exist
    if not os.path.exists(left_cam_directory):
        os.makedirs(left_cam_directory)
    if not os.path.exists(right_cam_directory):
        os.makedirs(right_cam_directory)


    capture_left = cv2.VideoCapture(CAPTURE_PIPE_l, cv2.CAP_GSTREAMER)
    capture_right = cv2.VideoCapture(CAPTURE_PIPE_r, cv2.CAP_GSTREAMER)
  

    video_writer_left  = NONE
    video_writer_right = NONE

    # capture_left.set(cv2.CAP_PROP_FPS, video_fps)
    # capture_right.set(cv2.CAP_PROP_FPS, video_fps)

    try:
        while True:     

            server = ThreadedHTTPServer(('192.168.42.241', 8087), CamHandler)
            print( "server started")      
            server.serve_forever() 

            
    except KeyboardInterrupt:
            capture_left.release()
            capture_right.release()
            server.socket.close()
            print("Server stopped")


    

