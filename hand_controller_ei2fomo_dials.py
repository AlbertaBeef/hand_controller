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
# Hand Controller with Visual Control Dials
#
# Based on:
#   https://github.com/ljkeller/visual_control/blob/master/proto.py
#
# References:
#   https://studio.edgeimpulse.com/studio/726391
#
# Dependencies:
#   Edge Impulse
#      edge_impulse_linux
#

app_name = "hand_controller_ei2fomo_dials"

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

from visualization import draw_stacked_bar_chart, stacked_bar_performance_colors
from visualization import tria_blue, tria_yellow, tria_pink, tria_aqua
from utils_linux import get_media_dev_by_name, get_video_dev_by_name

from edge_impulse_linux.image import ImageImpulseRunner

from timeit import default_timer as timer

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
#CAMERA_WIDTH = 1280
#CAMERA_HEIGHT = 720

stacked_bar_latency_colors = [
    tria_blue  , # resize
    tria_yellow, # fomo_pre
    tria_pink  , # fomo_model
    tria_aqua  , # fomo_post
    tria_blue  , # annotate
    tria_yellow, # dials
]
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
ap.add_argument('-m', '--model'      , type=str, default="./ei_fomo_face_hands_float32.eim", help='Path of Edge Impulse FOMO model. Default is ./ei_fomo_face_hands_float32.eim')
ap.add_argument('-v', '--verbose'    , default=False, action='store_true', help="Enable Verbose mode. Default is off")
ap.add_argument('-w', '--withoutview', default=False, action='store_true', help="Disable Output viewing. Default is on")
ap.add_argument('-z', '--profilelog' , default=False, action='store_true', help="Enable Profile Log (Latency). Default is off")
ap.add_argument('-y', '--profileview', default=False, action='store_true', help="Enable Profile View (Latency). Default is off")
ap.add_argument('-f', '--fps'        , default=False, action='store_true', help="Enable FPS display. Default is off")

args = ap.parse_args()  
  
print('Command line options:')
print(' --input       : ', args.input)
print(' --model       : ', args.model)
print(' --verbose     : ', args.verbose)
print(' --withoutview : ', args.withoutview)
print(' --profilelog  : ', args.profilelog)
print(' --profileview : ', args.profileview)
print(' --fps         : ', args.fps)


bInputImage = False
bInputVideo = False
bInputCamera = True

if os.path.exists(args.input):
    print("[INFO] Input exists : ",args.input)
    file_name, file_extension = os.path.splitext(args.input)
    file_extension = file_extension.lower()
    print("[INFO] Input type : ",file_extension)
    if file_extension == ".jpg" or file_extension == ".png" or file_extension == ".tif":
        bInputImage = True
        bInputVideo = False
        bInputCamera = False
    if file_extension == ".mov" or file_extension == ".mp4":
        bInputImage = False
        bInputVideo = True
        bInputCamera = False

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

if bInputVideo == True:
    # Open video file
    cap = cv2.VideoCapture(args.input)
    frame_width = int(round(cap.get(cv2.CAP_PROP_FRAME_WIDTH)))
    frame_height = int(round(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    print("[INFO] input : video ",args.input," (",frame_width,",",frame_height,")")

if bInputImage == True:
    image = cv2.imread(args.input)
    frame_height,frame_width,_ = image.shape
    print("[INFO] input : image ",args.input," (",frame_width,",",frame_height,")")

# Output directory for captured images
output_dir = './captured-images'
if not os.path.exists(output_dir):
    # Create the output directory if it doesn't already exist      
    os.mkdir(output_dir)            

# Profiling output
profile_csv = "./"+app_name+"_profiling.csv"
if os.path.isfile(profile_csv):
    f_profile_csv = open(profile_csv, "a")
    print("[INFO] Appending to existing profiling results file :",profile_csv)
else:
    f_profile_csv = open(profile_csv, "w")
    print("[INFO] Creating new profiling results file :",profile_csv)
    f_profile_csv.write("time,user,hostname,pipeline,detection-qty,resize,fomo_pre,fomo_model,fomo_post,annotate,dials,total,fps\n")

pipeline = app_name

# Visual Control Dials

CV_DRAW_COLOR_PRIMARY = tria_aqua

CONTROL_CIRCLE_DEADZONE_R = 50

@dataclass
class HandData:
    handedness: str
    landmarks: list
    center_perc: tuple

    def __init__(self, handedness, landmarks, image_width, image_height):
        self.handedness = handedness
        self.landmarks = landmarks.copy()
        self.landmarks[:,0] = self.landmarks[:,0] / image_width
        self.landmarks[:,1] = self.landmarks[:,1] / image_height        
        landmarks_len = landmarks.shape[0]
        x_avg = sum(self.landmarks[:,0]) / landmarks_len
        y_avg = sum(self.landmarks[:,1]) / landmarks_len
        z_avg = sum(self.landmarks[:,2]) / landmarks_len
        
        self.center_perc = (x_avg, y_avg, z_avg)
        #print(f"center_perc: {self.center_perc}")

def draw_control_overlay(img, lh_data=None, rh_data=None):
    CAMERA_HEIGHT, CAMERA_WIDTH, _ = img.shape

    CONTROL_CIRCLE_XY_CENTER = (int(CAMERA_WIDTH/4), int(CAMERA_HEIGHT/2))
    CONTROL_CIRCLE_Z_APERATURE_CENTER = (int(3*CAMERA_WIDTH/4), int(CAMERA_HEIGHT/2))
    
    # Draw control circle for XY control (left hand)
    cv2.circle(img, CONTROL_CIRCLE_XY_CENTER,
               CONTROL_CIRCLE_DEADZONE_R, CV_DRAW_COLOR_PRIMARY, 2)

    center_xy_point = CONTROL_CIRCLE_XY_CENTER
    hand_xy_point = CONTROL_CIRCLE_XY_CENTER # until proven otherwise

    if lh_data:
        # Normalize and compute actual pixel position of left hand
        xy_ctl_x_pct_normalized = min((lh_data.center_perc[0] - 0.25) * 4, 1.0)
        xy_ctl_y_pct_normalized = min((lh_data.center_perc[1] - 0.5) * 2, 1.0)

        xy_ctl_x = int(xy_ctl_x_pct_normalized *
                       CONTROL_CIRCLE_DEADZONE_R) + CONTROL_CIRCLE_XY_CENTER[0]
        xy_ctl_y = int(xy_ctl_y_pct_normalized *
                       CONTROL_CIRCLE_DEADZONE_R) + CONTROL_CIRCLE_XY_CENTER[1]

        hand_xy_point = (xy_ctl_x, xy_ctl_y)

        # Draw line from center to hand position
        cv2.line(img, center_xy_point, hand_xy_point,
                 CV_DRAW_COLOR_PRIMARY, 1)

        # Draw hand position dot
        cv2.circle(img, hand_xy_point, 4, CV_DRAW_COLOR_PRIMARY, cv2.FILLED)

    # Calculate normalized delta values
    delta_xy = tuple(np.subtract(center_xy_point,hand_xy_point))
    delta_xy = tuple(c/CONTROL_CIRCLE_DEADZONE_R for c in delta_xy)

    # Draw control circle for Z-aperture (right hand)
    cv2.circle(img, CONTROL_CIRCLE_Z_APERATURE_CENTER,
               CONTROL_CIRCLE_DEADZONE_R, CV_DRAW_COLOR_PRIMARY, 2)

    center_z_point = CONTROL_CIRCLE_Z_APERATURE_CENTER
    hand_z_point = CONTROL_CIRCLE_Z_APERATURE_CENTER # until proven otherwise

    if rh_data:
        z_ctl_pct_normalized = min((rh_data.center_perc[1] - 0.50) * 2, 1.0)
        aperature_ctl_x_pct_normalized = min(
            (rh_data.center_perc[0] - 0.75) * 4, 1.0)

        aperature_ctl_x = int(aperature_ctl_x_pct_normalized *
                              CONTROL_CIRCLE_DEADZONE_R) + CONTROL_CIRCLE_Z_APERATURE_CENTER[0]
        z_ctl_y = int(z_ctl_pct_normalized *
                      CONTROL_CIRCLE_DEADZONE_R) + CONTROL_CIRCLE_Z_APERATURE_CENTER[1]

        hand_z_point = (aperature_ctl_x, z_ctl_y)

        # Draw line from center to hand Z-position
        cv2.line(img, center_z_point, hand_z_point,
                 CV_DRAW_COLOR_PRIMARY, 1)

        # Draw hand position dot
        cv2.circle(img, hand_z_point, 4, CV_DRAW_COLOR_PRIMARY, cv2.FILLED)

    # Calculate normalized delta values
    delta_z = tuple(np.subtract(center_z_point,hand_z_point))
    delta_z = tuple(c/CONTROL_CIRCLE_DEADZONE_R for c in delta_z)

    # Optional: draw vertical center reference line
    cv2.line(img, (int(CAMERA_WIDTH / 2), 0),
             (int(CAMERA_WIDTH / 2), CAMERA_HEIGHT), CV_DRAW_COLOR_PRIMARY, 1)

    return delta_xy, delta_z       
        
print("================================================================")
print("Hand Controller (Edge Impulse) with Dials")
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
bMirrorImage = True
bShowFPS = args.fps
bVerbose = args.verbose
bViewOutput = not args.withoutview
bProfileLog = args.profilelog
bProfileView = args.profileview

print("[INFO] Mirror Image = ",bMirrorImage)

def ignore(x):
    pass

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

modelfile = args.model
if bVerbose:
    print("[INFO] model= ",modelfile)

with ImageImpulseRunner(modelfile) as runner:
 try:
  model_info = runner.init()
  if bVerbose:
      # displays WAY TOO MUCH verbose ... :( ...
      #model_info = runner.init(debug=True) # to get debug print out

      print('[INFO] Loaded runner for "' + model_info['project']['owner'] + ' / ' + model_info['project']['name'] + '"')
      
  labels = model_info['model_parameters']['labels'] 
  if bVerbose:
      print("[INFO] labels = ",labels)
 
 
  model_input_width = model_info['model_parameters']['image_input_width']
  model_input_height = model_info['model_parameters']['image_input_height']
  if bVerbose:
      print("[INFO] model input = ",model_input_width,"x",model_input_height)
  
  
  while True:
  
    # init the real-time FPS counter
    if rt_fps_count == 0:
        rt_fps_time = cv2.getTickCount()

    frame_count = frame_count + 1

    #if bUseImage:
    #    frame = cv2.imread('../woman_hands.jpg')
    #    if not (type(frame) is np.ndarray):
    #        print("[ERROR] cv2.imread('woman_hands.jpg') FAILED !")
    #        break;
    #elif bInputImage:
    if bInputImage:
        frame = cv2.imread(args.input)
        if not (type(frame) is np.ndarray):
            print("[ERROR] cv2.imread(",args.input,") FAILED !")
            break;
    else:
        flag, frame = cap.read()
        if not flag:
            print("[ERROR] cap.read() FAILEd !")
            break

    if bMirrorImage == True:
        # Mirror horizontally for selfie-mode
        frame = cv2.flip(frame, 1)        
        

    #image = cv2.resize(frame,(0,0), fx=scale, fy=scale) 
    image = frame
    output = image.copy()
    
    #            
    # Visual Control Dials (init hand data)
    #

    lh_data, rh_data = None, None
    
    #
    # Profiling
    #

    profile_resize         = 0
    profile_fomo_qty       = 0
    profile_fomo_pre       = 0
    profile_fomo_model     = 0
    profile_fomo_post      = 0
    profile_annotate       = 0
    profile_dials          = 0
    #
    profile_total          = 0
    profile_fps            = 0
    #
    profile_latency_title     = "Latency (sec)"
    profile_performance_title = "Performance (FPS)"


    #            
    # EdgeImpulse FOMO pipeline
    #
    
    # Create square images for left | right sides
    start = timer()
    image = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
    image_size = min(CAMERA_WIDTH,CAMERA_HEIGHT)
    image_overlap = int(image_size - (CAMERA_WIDTH/2))
    #print("[INFO] image.shape = ",image.shape )
    #print("[INFO] image_size = ",image_size )
    #print("[INFO] image_overlap = ",image_overlap )
    # [INFO] image.shape =  (480, 640, 3)
    # [INFO] image_size =  480
    # [INFO] image_overlap =  160
    image_l = image[:,0:image_size,:].copy()
    image_r = image[:,(CAMERA_WIDTH-image_size-1):-1,:].copy()
    #print("[INFO] image_l.shape = ",image_l.shape )
    #print("[INFO] image_r.shape = ",image_r.shape )
    # [INFO] image_l.shape =  (480, 480, 3)
    # [INFO] image_r.shape =  (480, 480, 3)
    # blank out overlapping portions
    image_l[:,(image_size-image_overlap-1):-1,:] = 0
    image_r[:,0:(image_overlap),:] = 0
    # resize to MODEL input size
    cropped_size = model_input_width
    image_l = cv2.resize(image_l,(cropped_size,cropped_size))
    image_r = cv2.resize(image_r,(cropped_size,cropped_size))    
    #print("[INFO] image_l.shape = ",image_l.shape )
    #print("[INFO] image_r.shape = ",image_r.shape )
    # [INFO] image_l.shape =  (160, 160, 3)
    # [INFO] image_r.shape =  (160, 160, 3)
    profile_resize = timer()-start

    # generate features
    start = timer()
    features_l, cropped_l = runner.get_features_from_image(image_l)
    features_r, cropped_r = runner.get_features_from_image(image_r)
    cropped_l = cv2.cvtColor(cropped_l,cv2.COLOR_RGB2BGR)
    cropped_r = cv2.cvtColor(cropped_r,cv2.COLOR_RGB2BGR)
    profile_fomo_pre = timer()-start
    
    # detection objects (FOMO)
    start = timer()
    res_l = runner.classify(features_l)
    res_r = runner.classify(features_r)
    profile_fomo_model = timer()-start

    #print("[INFO] res_l = ",res_l["result"].keys() )
    #print("[INFO] res_r = ",res_r["result"].keys() )    
    # [INFO] res_l =  dict_keys(['bounding_boxes'])
    # [INFO] res_r =  dict_keys(['bounding_boxes'])
        
    profile_fomo_qty = len(res_l)+len(res_r)

    if profile_fomo_qty > 0:

        #
        # Visual Control Dials (prepare hand data)
        # 

        # Process left side
        if "bounding_boxes" in res_l["result"].keys():
            if bVerbose:
                print('Found %d bounding boxes (%f ms, %f ms)' % (len(res_l["result"]["bounding_boxes"]), res_l['timing']['dsp'], res_l['timing']['classification']))

            for bb in res_l["result"]["bounding_boxes"]:
                if bVerbose:
                    print('\t%s (%.2f): x=%d y=%d w=%d h=%d' % (bb['label'], bb['value'], bb['x'], bb['y'], bb['width'], bb['height']))
                
                if bb['label'] == 'face':
                    continue
                    
                if bb['label'] == 'open':    
                    cropped_l = cv2.rectangle(cropped_l, (bb['x'], bb['y']), (bb['x'] + bb['width'], bb['y'] + bb['height']), (0, 255, 0), 2)
                if bb['label'] == 'closed':    
                    cropped_l = cv2.rectangle(cropped_l, (bb['x'], bb['y']), (bb['x'] + bb['width'], bb['y'] + bb['height']), (0, 0, 255), 2)

                # Visual Control Dials (prepare hand data)
                start = timer()    
                handedness = "Left"
                x1 = (bb['x'] / cropped_size) * image_size
                y1 = (bb['y'] / cropped_size) * image_size
                z1 = 0.0
                x2 = ((bb['x'] + bb['width']) / cropped_size) * image_size
                y2 = ((bb['y'] + bb['height']) / cropped_size) * image_size
                z2 = 0.0
                landmarks = np.asarray([[x1,y1,z1],[x2,y2,z2]])
                lh_data = HandData(handedness, landmarks, CAMERA_WIDTH, CAMERA_HEIGHT)
                profile_dials += timer()-start

                if bb['label'] == 'open':
                    output = cv2.rectangle(output, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                if bb['label'] == 'closed':    
                    output = cv2.rectangle(output, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)

        # Process right side
        if "bounding_boxes" in res_r["result"].keys():
            if bVerbose:
                print('Found %d bounding boxes (%f ms, %f ms)' % (len(res_r["result"]["bounding_boxes"]), res_r['timing']['dsp'], res_r['timing']['classification']))

            for bb in res_r["result"]["bounding_boxes"]:
                if bVerbose:
                    print('\t%s (%.2f): x=%d y=%d w=%d h=%d' % (bb['label'], bb['value'], bb['x'], bb['y'], bb['width'], bb['height']))

                if bb['label'] == 'open':    
                    cropped_r = cv2.rectangle(cropped_r, (bb['x'], bb['y']), (bb['x'] + bb['width'], bb['y'] + bb['height']), (0, 255, 0), 2)
                if bb['label'] == 'closed':    
                    cropped_r = cv2.rectangle(cropped_r, (bb['x'], bb['y']), (bb['x'] + bb['width'], bb['y'] + bb['height']), (0, 0, 255), 2)
                
                # Visual Control Dials (prepare hand data)
                start = timer()    
                handedness = "Right"
                x1 = CAMERA_WIDTH - image_size + (bb['x'] / cropped_size) * image_size
                y1 = (bb['y'] / cropped_size) * image_size
                z1 = 0.0
                x2 = CAMERA_WIDTH - image_size + ((bb['x'] + bb['width']) / cropped_size) * image_size
                y2 = ((bb['y'] + bb['height']) / cropped_size) * image_size
                z2 = 0.0
                landmarks = np.asarray([[x1,y1,z1],[x2,y2,z2]])
                rh_data = HandData(handedness, landmarks, CAMERA_WIDTH, CAMERA_HEIGHT)
                profile_dials += timer()-start

                if bb['label'] == 'open':
                    output = cv2.rectangle(output, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                if bb['label'] == 'closed':    
                    output = cv2.rectangle(output, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)
                
    cropped_output = cv2.hconcat([cropped_l,cropped_r])
    #print("[INFO] cropped_output.shape = ",cropped_output.shape )
    # [INFO] cropped_output.shape =  (160, 320, 3)
    cropped_output[:,cropped_size:cropped_size+1] = tria_aqua # create middle bondary
    if bViewOutput:    
        cv2.imshow('FOMO input (left|right)', cropped_output)

    # Visual Control Dials (display dials)
    start = timer()
    if lh_data:
        cv2.circle(output, (int(lh_data.center_perc[0]*CAMERA_WIDTH), int(lh_data.center_perc[1]*CAMERA_HEIGHT)), radius=10, color=tria_pink, thickness=-1)  
    if rh_data:
        cv2.circle(output, (int(rh_data.center_perc[0]*CAMERA_WIDTH), int(rh_data.center_perc[1]*CAMERA_HEIGHT)), radius=10, color=tria_pink, thickness=-1)
    delta_xy, delta_z = draw_control_overlay(output, lh_data, rh_data)
    profile_dials += timer()-start
    print(f"[INFO] DIALS XY={delta_xy[0]:+.3f}|{delta_xy[1]:+.3f}, Z={delta_z[0]:+.3f}|{delta_z[1]:+.3f}")

    # display real-time FPS counter (if valid)
    if rt_fps_valid == True and bShowFPS:
        cv2.putText(output,rt_fps_message, (rt_fps_x,rt_fps_y),text_fontType,text_fontSize,text_color,text_lineSize,text_lineType)
        if not bViewOutput:
            print("[REAL-TIME]",rt_fps_message)

    #
    # Profiling
    #
    profile_fomo = profile_fomo_pre + profile_fomo_model + profile_fomo_post
    profile_total = profile_resize + \
                    profile_fomo + \
                    profile_annotate + \
                    profile_dials
    profile_fps = 1.0 / profile_total
    if bProfileLog == True:
        # display profiling results to console
        print(f"[PROFILING] fomo_qty={profile_fomo_qty}, FPS={profile_fps:.3f}fps, Total={profile_total*1000:.3f}ms, FOMO={profile_fomo*1000:.3f}ms, Annotate={profile_annotate*1000:.3f}ms, DIALS={profile_dials*1000:.3f}ms")
        # write profiling results to csv file
        timestamp = datetime.now()
        csv_str = \
            str(timestamp)+","+\
            str(user)+","+\
            str(host)+","+\
            pipeline+","+\
            str(profile_fomo_qty)+","+\
            str(profile_resize)+","+\
            str(profile_fomo_pre)+","+\
            str(profile_fomo_model)+","+\
            str(profile_fomo_post)+","+\
            str(profile_annotate)+","+\
            str(profile_dials)+","+\
            str(profile_total)+","+\
            str(profile_fps)+"\n"
        f_profile_csv.write(csv_str)

    #
    # Annotated Output
    #
    
    if bViewOutput:
        # show the output image
        cv2.imshow(app_main_title, output)

    if bWrite:
        filename = ("%s_frame%04d_input.tif"%(app_name,frame_count))
        print("Capturing ",filename," ...")
        input_img = cv2.cvtColor(image,cv2.COLOR_RGB2BGR)
        cv2.imwrite(os.path.join(output_dir,filename),input_img)

        filename = ("%s_frame%04d_annotations.tif"%(app_name,frame_count))
        print("Capturing ",filename," ...")
        cv2.imwrite(os.path.join(output_dir,filename),output)

        filename = ("%s_frame%04d_debug.tif"%(app_name,frame_count))
        print("Capturing ",filename," ...")
        cv2.imwrite(os.path.join(output_dir,filename),cropped_output)
        

    if bProfileView:
        #
        # Latency
        #
        component_labels = [
            "resize",
            "fomo[pre]",
            "fomo[model]",
            "fomo[post]",
            "annotate",
            "dials"
        ]
        pipeline_titles = [app_name]
        component_values=[
            [profile_resize],
            [profile_fomo_pre],
            [profile_fomo_model],
            [profile_fomo_post],
            [profile_annotate],
            [profile_dials]
        ]
        profile_latency_chart = draw_stacked_bar_chart(
            pipeline_titles=pipeline_titles,
            component_labels=component_labels,
            component_values=component_values,
            component_colors=stacked_bar_latency_colors,
            chart_name=profile_latency_title
        )

        # Display or process the image using OpenCV or any other library
        if bViewOutput:
            cv2.imshow(profile_latency_title, profile_latency_chart)                         

        if bWrite:
            filename = ("%s_frame%04d_profiling_latency.png"%(app_name,frame_count))
            print("Capturing ",filename," ...")
            cv2.imwrite(os.path.join(output_dir,filename),profile_latency_chart)

        #
        # FPS
        #

        component_labels = [
            "fps"
        ]
        component_values=[
            [profile_fps]
        ]        
        profile_performance_chart = draw_stacked_bar_chart(
            pipeline_titles=pipeline_titles,
            component_labels=component_labels,
            component_values=component_values,
            component_colors=stacked_bar_performance_colors,
            chart_name=profile_performance_title
        )

        # Display or process the image using OpenCV or any other library
        if bViewOutput:
            cv2.imshow(profile_performance_title, profile_performance_chart)                         

        if bWrite:
            filename = ("%s_frame%04d_profiling_performance.png"%(app_name,frame_count))
            print("Capturing ",filename," ...")
            cv2.imwrite(os.path.join(output_dir,filename),profile_performance_chart)


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

    if key == 118: # 'v'
        bVerbose = not bVerbose
        print("[INFO] Verbose = ",bVerbose)

    if key == 122: # 'z'
        bProfileLog = not bProfileLog
        print("[INFO] bProfileLog=",bProfileLog)

    if key == 121: # 'y'
        bProfileView = not bProfileView 
        print("[INFO] bProfileView=",bProfileView)
        if not bProfileView:
            cv2.destroyWindow(profile_latency_title)
            cv2.destroyWindow(profile_performance_title)

    if key == 27 or key == 113: # ESC or 'q':
        break

    # automated test/profiling mode
    if not bViewOutput and frame_count==100:
        bWrite = True
    if not bViewOutput and frame_count==101:
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

 finally:
    if (runner):
        runner.stop()
        
# Cleanup
f_profile_csv.close()
cv2.destroyAllWindows()
