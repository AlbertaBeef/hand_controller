'''
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
    http://www.apache.org/licenses/LICENSE-2.0
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
'''
#
# Hand Controller Recorder
#
#

app_name = "hand_controller_recorder"

import numpy as np
import cv2
import os
from datetime import datetime
import itertools

from dataclasses import dataclass
from ctypes import *
from typing import List
import pathlib
#import threading
import time
import sys
import argparse
import glob
import subprocess
import re
import sys

import getpass
import socket
user = getpass.getuser()
host = socket.gethostname()
user_host_descriptor = user+"@"+host
print("[INFO] user@hosthame : ",user_host_descriptor)

sys.path.append(os.path.abspath('blaze_app_python/'))
sys.path.append(os.path.abspath('blaze_app_python/blaze_common/'))

from utils_linux import get_media_dev_by_name, get_video_dev_by_name

from timeit import default_timer as timer

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
#CAMERA_WIDTH = 1280
#CAMERA_HEIGHT = 720

# Parameters (tweaked for video)
scale = 1.0
text_fontType = cv2.FONT_HERSHEY_SIMPLEX
text_fontSize = 0.75*scale
text_color    = (0,0,255)
text_lineSize = max( 1, int(2*scale) )
text_lineType = cv2.LINE_AA

# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument('-i', '--input'      , type=str, default="", help="Video input device. Default is auto-detect (first usbcam)")
ap.add_argument('-o', '--output'     , type=str, default="hand_controller_recorder.mp4", help="Video output file. Default is hand_controller_recorder.mp4")
ap.add_argument('-v', '--verbose'    , default=False, action='store_true', help="Enable Verbose mode. Default is off")
ap.add_argument('-w', '--withoutview', default=False, action='store_true', help="Disable Output viewing. Default is on")
ap.add_argument('-f', '--fps'        , default=False, action='store_true', help="Enable FPS display. Default is off")

args = ap.parse_args()  
  
print('Command line options:')
print(' --input       : ', args.input)
print(' --output      : ', args.output)
print(' --verbose     : ', args.verbose)
print(' --withoutview : ', args.withoutview)
print(' --fps         : ', args.fps)

bInputCamera = True

if bInputCamera == True:
    print("[INFO] Searching for USB camera ...")
    dev_video = get_video_dev_by_name("uvcvideo")
    dev_media = get_media_dev_by_name("uvcvideo")
    print(dev_video)
    print(dev_media)

    if dev_video == None:
        input_video = 0
    elif args.input != "":
        input_video = args.input 
    else:
        input_video = dev_video  

    # Open video
    cap = cv2.VideoCapture(input_video)
    frame_width = CAMERA_WIDTH
    frame_height = CAMERA_HEIGHT
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,frame_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,frame_height)
    #frame_width = int(round(cap.get(cv2.CAP_PROP_FRAME_WIDTH)))
    #frame_height = int(round(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    print("[INFO] input : camera",input_video," (",frame_width,",",frame_height,")")

# Output video file for recording
output_video_file = args.output
if os.path.isfile(output_video_file):
    print("[ERROR] output file exists !")
    exit()
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter(output_video_file, fourcc, 30, (CAMERA_WIDTH, CAMERA_HEIGHT))
          
        
print("================================================================")
print("Hand Controller Recorder")
print("================================================================")
print("\tPress ESC to quit ...")
print("----------------------------------------------------------------")
print("\tPress 'p' to pause video ...")
print("\tPress 'c' to continue ...")
print("\tPress 's' to step one frame at a time ...")
print("\tPress 'w' to take a photo ...")
print("----------------------------------------------------------------")
print("\tPress 'h' to toggle horizontal mirror on input")
print("\tPress 'f' to toggle FPS display on/off")
print("\tPress 'v' to toggle verbose on/off")
print("\tPress 'z' to toggle profiling log on/off")
print("\tPress 'y' to toggle profiling view on/off")
print("================================================================")

bStep = False
bPause = False
bWrite = False
bMirrorImage = False
bShowFPS = args.fps
bVerbose = args.verbose
bViewOutput = not args.withoutview


print("[INFO] Mirror Image = ",bMirrorImage)

if bViewOutput:
    app_main_title = "Hand Controller Demo"
    app_ctrl_title = "Hand Controller Demo"

    cv2.namedWindow(app_main_title)

image = []
output = []

frame_count = 0

# init the real-time FPS counter
rt_fps_count = 0
rt_fps_time = cv2.getTickCount()
rt_fps_valid = False
rt_fps = 0.0
rt_fps_message = "FPS: {0:.2f}".format(rt_fps)
rt_fps_x = int(10*scale)
rt_fps_y = int((frame_height-10)*scale)

 
while True:
  
    # init the real-time FPS counter
    if rt_fps_count == 0:
        rt_fps_time = cv2.getTickCount()

    frame_count = frame_count + 1

    flag, frame = cap.read()
    if not flag:
        print("[ERROR] cap.read() FAILED !")
        break

    if bMirrorImage == True:
        # Mirror horizontally for selfie-mode
        frame = cv2.flip(frame, 1)        
        

    #image = cv2.resize(frame,(0,0), fx=scale, fy=scale) 
    image = frame
    output = image.copy()
    

    # display real-time FPS counter (if valid)
    if rt_fps_valid == True and bShowFPS:
        cv2.putText(output,rt_fps_message, (rt_fps_x,rt_fps_y),text_fontType,text_fontSize,text_color,text_lineSize,text_lineType)
        if not bViewOutput:
            print("[REAL-TIME]",rt_fps_message)


    #
    # Annotated Output
    #
    
    if True:
        # show the output image
        cv2.imshow(app_main_title, output)

    out.write(output)

    #
    # Keyboard Control
    #
         
    if bStep == True:
        key = cv2.waitKey(0)
    elif bPause == True:
        key = cv2.waitKey(0)
    else:
        key = cv2.waitKey(1)

    #print(key)
    
    bWrite = False
    if key == 119: # 'w'
        bWrite = True

    if key == 115: # 's'
        bStep = True    
    
    if key == 112: # 'p'
        bPause = not bPause

    if key == 99: # 'c'
        bStep = False
        bPause = False

    if key == 104: # 'h'
        bMirrorImage = not bMirrorImage  
        print("[INFO] bMirrorImage=",bMirrorImage)

    if key == 102: # 'f'
        bShowFPS = not bShowFPS
        print("[INFO] bShowFPS=",bShowFPS)

    if key == 27 or key == 113: # ESC or 'q':
        break

    # Update the real-time FPS counter
    rt_fps_count = rt_fps_count + 1
    if rt_fps_count == 10:
        t = (cv2.getTickCount() - rt_fps_time)/cv2.getTickFrequency()
        rt_fps_valid = 1
        rt_fps = 10.0/t
        rt_fps_message = "FPS: {0:.2f}".format(rt_fps)
        #print("[INFO] ",rt_fps_message)
        rt_fps_count = 0

        
# Cleanup
cap.release()
out.release()
cv2.destroyAllWindows()

