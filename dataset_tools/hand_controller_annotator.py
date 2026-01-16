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

# MediaPipe
import mediapipe as mp

mp_holistic = mp.solutions.holistic
holistic_model = mp_holistic.Holistic(
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5
)

mp_drawing = mp.solutions.drawing_utils

def mediapipe_multi_landmark_to_roi(multi_landmarks):
    bboxes = []
    for lm_id in range(len(multi_landmarks)):
        landmarks = multi_landmarks[lm_id]
                
        # Extract coordinates
        points_raw=[]
        for lm in landmarks.landmark:
            points_raw.append([lm.x, lm.y, lm.z])
        points_raw = np.array(points_raw)

        # Find region of interest
        min_x = np.min(points_raw[:, 0])
        max_x = np.max(points_raw[:, 0])
        min_y = np.min(points_raw[:, 1])
        max_y = np.max(points_raw[:, 1])

        bbox = [min_x,min_y,max_x,max_y]
        bboxes.append(bbox)

    return bboxes

def mediapipe_landmark_to_roi(landmarks):
                
    # Extract coordinates
    points_raw=[]
    for lm in landmarks.landmark:
        points_raw.append([lm.x, lm.y, lm.z])
    points_raw = np.array(points_raw)

    # Find region of interest
    min_x = np.min(points_raw[:, 0])
    max_x = np.max(points_raw[:, 0])
    min_y = np.min(points_raw[:, 1])
    max_y = np.max(points_raw[:, 1])

    bbox = [min_x,min_y,max_x,max_y]

    return bbox

def is_hand_closed(hand_landmarks):
    # Indices: 0 = wrist, 4 = thumb tip, 8 = index tip, 12 = middle tip, 16 = ring tip, 20 = pinky tip
    closed_fingers = 0
    for tip_idx, base_idx in zip([4, 8, 12, 16, 20], [3, 6, 10, 14, 18]):
        tip = hand_landmarks.landmark[tip_idx]
        base = hand_landmarks.landmark[base_idx]
        wrist = hand_landmarks.landmark[0]
        # Calculate distance from wrist to fingertip and base
        tip_dist = np.linalg.norm(np.array([tip.x, tip.y]) - np.array([wrist.x, wrist.y]))
        base_dist = np.linalg.norm(np.array([base.x, base.y]) - np.array([wrist.x, wrist.y]))
        # If tip is closer to wrist than to base, finger is closed
        if tip_dist < base_dist:
            closed_fingers += 1
    # If most fingers are closed, consider hand closed
    return closed_fingers > 3

def resize_pad(img,h_scale,w_scale):
    """ resize and pad images to be input to the detectors

    The FOMO model take 160x160 images as input. 
    As such the input image is padded and resized to fit the
    size while maintaing the aspect ratio.

    Returns:
        img: HxW
        scale: scale factor between original image and 160x160 image
        pad: pixels of padding in the original image
    """

    size0 = img.shape
    if size0[0] >= size0[1]:
        h1 = int(h_scale)
        w1 = int(w_scale * size0[1] // size0[0])
        padh = 0
        padw = int(w_scale - w1)
        scale = size0[1] / w1
    else:
        h1 = int(h_scale * size0[0] // size0[1])
        w1 = int(w_scale)
        padh = int(h_scale - h1)
        padw = 0
        scale = size0[0] / h1
    padh1 = padh//2
    padh2 = padh//2 + padh % 2
    padw1 = padw//2
    padw2 = padw//2 + padw % 2
    img = cv2.resize(img, (w1, h1))
    img = np.pad(img, ((padh1, padh2), (padw1, padw2), (0, 0)), mode='constant')
    pad = (int(padh1 * scale), int(padw1 * scale))
    return img, scale, pad

sys.path.append(os.path.abspath('blaze_app_python/'))
sys.path.append(os.path.abspath('blaze_app_python/blaze_common/'))

from visualization import tria_pink
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
ap.add_argument('-i', '--input'      , type=str, default="", help="Video input file.")
ap.add_argument('-o', '--output'     , type=str, default="hand_controller_annotations", help="Annotations output directory. Default is hand_controller_annotations")
ap.add_argument('-t', '--type'       , type=str, default="edgeimpulse", help="Annotation type. Default is edgeimpulse")
ap.add_argument('-n', '--name'       , type=str, default="hand_controller_annotations", help="Name of Annotations. Default is hand_controller_annotations")
ap.add_argument('-v', '--verbose'    , default=False, action='store_true', help="Enable Verbose mode. Default is off")
ap.add_argument('-w', '--withoutview', default=False, action='store_true', help="Disable Output viewing. Default is on")
ap.add_argument('-f', '--fps'        , default=False, action='store_true', help="Enable FPS display. Default is off")
ap.add_argument('-s', '--skip_frames', default=0, help="Number of frames to skip for processing.")
ap.add_argument('-m', '--max_frames' , default=100, help="Maximum number of frames to process.")

args = ap.parse_args()  
  
print('Command line options:')
print(' --input       : ', args.input)
print(' --output      : ', args.output)
print(' --name        : ', args.name)
print(' --type        : ', args.type)
print(' --verbose     : ', args.verbose)
print(' --withoutview : ', args.withoutview)
print(' --fps         : ', args.fps)
print(' --skip_frames : ', args.skip_frames)
print(' --max_frames  : ', args.max_frames)

bInputVideo = True

if bInputVideo == True:
    if not os.path.isfile(args.input):
        print("[ERROR] input video file does not exist !")
        exit()
        
    # Open video file
    cap = cv2.VideoCapture(args.input)
    frame_width = int(round(cap.get(cv2.CAP_PROP_FRAME_WIDTH)))
    frame_height = int(round(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    frame_count = cap.get(cv2.CAP_PROP_FRAME_COUNT)
    print("[INFO] input : video ",args.input," (",frame_width,",",frame_height,")",int(frame_count),"frames")


          
        
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

prev_frame_num = 0
frame_num = 0
num = 0
num_images = int(frame_count)

skip_frames = int(args.skip_frames)
max_frames  = int(args.max_frames)
print("[INFO]    Skip Frames   : ",skip_frames)
print("[INFO]    Max Frames    : ",max_frames)

def ignore(x):
    pass
    
if bViewOutput:
    app_main_title = "Hand Controller Demo"
    app_ctrl_title = "Hand Controller Demo"

    cv2.namedWindow(app_main_title)
    
    cv2.createTrackbar('frameNum'  , app_ctrl_title, 0, num_images-1, ignore)    

image = []
output = []

# Output directory
output_directory = args.output
print("[INFO] Output directory : ",output_directory)
    
# Training sub-directory
training_directory = os.path.join(output_directory,"training")
print("[INFO] Training directory : ",training_directory)

# Create the expected directory structure
if not os.path.exists(output_directory):
    os.makedirs(output_directory)
if not os.path.exists(training_directory):
    os.makedirs(training_directory)

    

# info.labels 
# reference : https://docs.edgeimpulse.com/tools/specifications/data-annotation/ei-labels
#{
#    "version": 1,
#    "files": [
#        {
#            "path": "testing/image_1.jpg",
#            "name": "image_1",
#            "category": "testing",
#            "label": {
#                "type": "label",
#                "label": "data_123"
#            },
#            "boundingBoxes": [
#                {
#                    "label": "object_1",
#                    "x": 606,
#                    "y": 103,
#                    "width": 32,
#                    "height": 17
#                }
#            ]
#        }
#    ]
#}
info_labels_filename = os.path.join(training_directory,"info.labels")
info_labels = open(info_labels_filename,"w")
info_labels.write('{\r\n')
info_labels.write('\t"version": 1,\r\n')
info_labels.write('\t"files": [\r\n')
#...
#info_labels.write('\t]\r\n')
#info_labels.write('}\r\n')

# 'bounding_boxes.labels


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

    cap.set(cv2.CAP_PROP_POS_FRAMES, num)
    num = num + (skip_frames+1)
    #print("[INFO] Frame Num : ",num)
    flag, frame = cap.read()
    if not flag:
        print("[ERROR] cap.read() FAILED !")
        break

    # Trackbar sliders
    frame_num = cv2.getTrackbarPos('frameNum', app_ctrl_title)
    if prev_frame_num != frame_num:
        frame_cnt = int(round(float(frame_num)/(skip_frames+1)))
        frame_num = frame_cnt * (skip_frames+1)
        print("[INFO] Frame Slider : ",frame_num)
        prev_frame_num = frame_num
        num = frame_num


    if bMirrorImage == True:
        # Mirror horizontally for selfie-mode
        frame = cv2.flip(frame, 1)        

    #image = cv2.resize(frame,(0,0), fx=scale, fy=scale) 
    #image = frame
    pad = np.zeros(shape=(80,640,3),dtype=np.uint8)
    image = cv2.vconcat([pad,frame,pad])
    image_width = 640
    image_height = 640
    
    output = image.copy()

    # Converting the from BGR to RGB
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    # Making predictions using holistic model
    # To improve performance, optionally mark the image as not writeable to
    # pass by reference.
    image_rgb.flags.writeable = False
    results = holistic_model.process(image_rgb)
    image_rgb.flags.writeable = True

    # Converting back the RGB image to BGR
    #image = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)

    #print("[INFO] results ",results)
    #print("[INFO] results.face_landmarks ",results.face_landmarks)
    #print("[INFO] results.right_hand_landmarks ",results.right_hand_landmarks)
    #print("[INFO] results.left_hand_landmarks ",results.left_hand_landmarks)

    bounding_boxes = []
    labels = []
    
    # Drawing the Facial Landmarks
    mp_drawing.draw_landmarks(
      output,
      results.face_landmarks,
      mp_holistic.FACEMESH_CONTOURS,
      mp_drawing.DrawingSpec(
        color=(255,0,255),
        thickness=1,
        circle_radius=1
      ),
      mp_drawing.DrawingSpec(
        color=(0,255,255),
        thickness=1,
        circle_radius=1
      )
    )
    if results.face_landmarks:
        bbox = mediapipe_landmark_to_roi(results.face_landmarks)
        x1 = int(bbox[0]*image_width)
        y1 = int(bbox[1]*image_height)
        x2 = int(bbox[2]*image_width)
        y2 = int(bbox[3]*image_height)
        w = x2-x1
        h = y2-y1
        cv2.rectangle(output,(x1,y1),(x2,y2),tria_pink,4)
        cv2.putText(output,"face",(x1,y1-10),cv2.FONT_HERSHEY_SIMPLEX, 0.8, tria_pink, 2, cv2.LINE_AA)
            
        bounding_boxes.append( [x1,y1,w,h] )
        labels.append( "face" )

    # Drawing Right hand Land Marks
    mp_drawing.draw_landmarks(
      output, 
      results.right_hand_landmarks, 
      mp_holistic.HAND_CONNECTIONS
    )
    if results.right_hand_landmarks:
        bbox = mediapipe_landmark_to_roi(results.right_hand_landmarks)
        x1 = int(bbox[0]*image_width)
        y1 = int(bbox[1]*image_height)
        x2 = int(bbox[2]*image_width)
        y2 = int(bbox[3]*image_height)
        w = x2-x1
        h = y2-y1
        cv2.rectangle(output,(x1,y1),(x2,y2),tria_pink,4)
        closed = is_hand_closed(results.right_hand_landmarks)
        if closed:
            cv2.putText(output,"closed",(x1,y1-10),cv2.FONT_HERSHEY_SIMPLEX, 0.8, tria_pink, 2, cv2.LINE_AA)
            bounding_boxes.append( [x1,y1,w,h] )
            labels.append( "closed" )
        else:
            cv2.putText(output,"open",(x1,y1-10),cv2.FONT_HERSHEY_SIMPLEX, 0.8, tria_pink, 2, cv2.LINE_AA)
            bounding_boxes.append( [x1,y1,w,h] )
            labels.append( "open" )

    # Drawing Left hand Land Marks
    mp_drawing.draw_landmarks(
      output, 
      results.left_hand_landmarks, 
      mp_holistic.HAND_CONNECTIONS
    )
    if results.left_hand_landmarks:
        bbox = mediapipe_landmark_to_roi(results.left_hand_landmarks)
        x1 = int(bbox[0]*image_width)
        y1 = int(bbox[1]*image_height)
        x2 = int(bbox[2]*image_width)
        y2 = int(bbox[3]*image_height)
        w = x2-x1
        h = y2-y1
        cv2.rectangle(output,(x1,y1),(x2,y2),tria_pink,4)
        closed = is_hand_closed(results.left_hand_landmarks)
        if closed:
            cv2.putText(output,"closed",(x1,y1-10),cv2.FONT_HERSHEY_SIMPLEX, 0.8, tria_pink, 2, cv2.LINE_AA)
            bounding_boxes.append( [x1,y1,w,h] )
            labels.append( "closed" )
        else:
            cv2.putText(output,"open",(x1,y1-10),cv2.FONT_HERSHEY_SIMPLEX, 0.8, tria_pink, 2, cv2.LINE_AA)
            bounding_boxes.append( [x1,y1,w,h] )
            labels.append( "open" )

    # Export Annotations
    if args.type == "edgeimpulse":
        name = f"{args.name}_frame{num:03d}"
        #print("[INFO] name = ", name)
 
        image_filename = f"{name}.jpg"
        #print("[INFO] image_filename = ",image_filename)

        image_fullpath = os.path.join(training_directory,image_filename)
        print("[INFO] image_fullpath = ",image_fullpath)
        
        cv2.imwrite(image_fullpath,image)
        
# info.labels 
# reference : https://docs.edgeimpulse.com/tools/specifications/data-annotation/ei-labels
#{
#    "version": 1,
#    "files": [
#        {
#            "path": "testing/image_1.jpg",
#            "name": "image_1",
#            "category": "testing",
#            "label": {
#                "type": "label",
#                "label": "data_123"
#            },
#            "boundingBoxes": [
#                {
#                    "label": "object_1",
#                    "x": 606,
#                    "y": 103,
#                    "width": 32,
#                    "height": 17
#                }
#            ]
#        }
#    ]
#}
        info_labels.write('\t\t{\r\n')
        info_labels.write(f'\t\t\t"path": "{image_filename}",\r\n')
        info_labels.write(f'\t\t\t"name": "{name}",\r\n')
        info_labels.write('\t\t\t"category": "testing",\r\n')
        #info_labels.write(f'\t\t\t"label": {{"type": "label", "label": "{name}"}},\r\n')
        info_labels.write(f'\t\t\t"label": {{"type": "unlabeled"}},\r\n')
        #"label":{"type":"unlabeled"},"metadata":{"labeled_by":"owlv2","prompt":"A person\\'s face (face, 0.1)"},            
        info_labels.write(f'\t\t\t"metadata":{{"labeled_by":"hand_controller_annotator.py"}},\r\n')
        info_labels.write('\t\t\t"boundingBoxes": [\r\n')
        for i in range(len(bounding_boxes)):
            bbox = bounding_boxes[i]
            label = labels[i]
            info_labels.write('\t\t\t\t{\r\n')
            info_labels.write(f'\t\t\t\t\t"label" : "{label}",\r\n')
            info_labels.write(f'\t\t\t\t\t"x" : {bbox[0]},\r\n')
            info_labels.write(f'\t\t\t\t\t"y" : {bbox[1]},\r\n')
            info_labels.write(f'\t\t\t\t\t"width" : {bbox[2]},\r\n')
            info_labels.write(f'\t\t\t\t\t"height" : {bbox[3]}\r\n')
            if i+1 < len(bounding_boxes):
                info_labels.write('\t\t\t\t},\r\n')
            else:
                info_labels.write('\t\t\t\t}\r\n')
        info_labels.write('\t\t\t]\r\n')
        info_labels.write('\t\t},\r\n')


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

    #
    # Keyboard Control
    #
         
    if bStep == True:
        key = cv2.waitKey(0)
    elif bPause == True:
        key = cv2.waitKey(33)
        num = num - (skip_frames+1)
    else:
        key = cv2.waitKey(33)

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

    frame_cnt = int(round(float(num)/(skip_frames+1)))

    #print("[INFO] Frame Qty : ",frame_cnt, max_frames)
    if frame_cnt >= max_frames:
        print("[INFO] Max Frames reached ... quitting")
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


info_labels.write('\t]\r\n')
info_labels.write('}\r\n')
info_labels.close()
        
# Cleanup
cap.release()
cv2.destroyAllWindows()

