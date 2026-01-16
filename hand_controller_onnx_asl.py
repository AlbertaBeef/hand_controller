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
# Hand Controller with ASL
#
# References:
#   https://www.github.com/AlbertaBeef/blaze_app_python
#   https://www.github.com/AlbertaBeef/hand_controller
#
# Dependencies:
#   Model conversion to ONNX
#      tf2onnx
#      onnxsim
#   Inference with ONNX
#      onnxruntime
#

app_name = "hand_controller_onnx_asl"

import numpy as np
import cv2
import os
from datetime import datetime
import itertools

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
#sys.path.append(os.path.abspath('blaze_app_python/blaze_tflite/'))
#sys.path.append(os.path.abspath('blaze_app_python/blaze_pytorch/'))
sys.path.append(os.path.abspath('blaze_app_python/blaze_onnx/'))
#sys.path.append(os.path.abspath('blaze_app_python/blaze_vitisai/'))
#sys.path.append(os.path.abspath('blaze_app_python/blaze_hailo/'))

#from blaze_tflite.blazedetector import BlazeDetector as BlazeDetector_tflite
#from blaze_tflite.blazelandmark import BlazeLandmark as BlazeLandmark_tflite
#from blaze_pytorch.blazedetector import BlazeDetector as BlazeDetector_pytorch
#from blaze_pytorch.blazelandmark import BlazeLandmark as BlazeLandmark_pytorch
from blaze_onnx.blazedetector import BlazeDetector as BlazeDetector_onnx
from blaze_onnx.blazelandmark import BlazeLandmark as BlazeLandmark_onnx

from visualization import draw_detections, draw_landmarks, draw_roi
from visualization import HAND_CONNECTIONS, FACE_CONNECTIONS, POSE_FULL_BODY_CONNECTIONS, POSE_UPPER_BODY_CONNECTIONS
from visualization import draw_detection_scores
from visualization import draw_stacked_bar_chart, stacked_bar_performance_colors
from visualization import tria_blue, tria_yellow, tria_pink, tria_aqua
from utils_linux import get_media_dev_by_name, get_video_dev_by_name

from timeit import default_timer as timer

import onnxruntime

model_path = './asl_pointnet'
model_name = 'asl_pointnet.onnx'
session = onnxruntime.InferenceSession(os.path.join(model_path, model_name))        

# reading onnx model parameters
session_inputs = session.get_inputs()
session_outputs = session.get_outputs()
num_inputs = len(session_inputs)
num_outputs = len(session_outputs)
if True:
   print("[BlazeDetector.load_model] Number of Inputs : ",num_inputs)
   for i in range(num_inputs):
       print("[BlazeDetector.load_model] Input[",i,"] Shape : ",session_inputs[i].shape," (",session_inputs[i].name,")")
   print("[BlazeDetector.load_model] Number of Outputs : ",num_outputs)
   for i in range(num_outputs):
       print("[BlazeDetector.load_model] Output[",i,"] Shape : ",session_outputs[i].shape," (",session_outputs[i].name,")")


char2int = {
            "A":0, "B":1, "C":2, "D":3, "E":4, "F":5, "G":6, "H":7, "I":8, "K":9, "L":10, "M":11,
            "N":12, "O":13, "P":14, "Q":15, "R":16, "S":17, "T":18, "U":19, "V":20, "W":21, "X":22, "Y":23
            }

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480

stacked_bar_latency_colors = [
    tria_blue  , # resize
    tria_yellow, # detector_pre
    tria_pink  , # detector_model
    tria_aqua  , # detector_post
    tria_blue  , # extract_roi
    tria_yellow, # landmark_pre
    tria_pink  , # landmark_model
    tria_aqua  , # landmark_post
    tria_blue  , # annotate
    tria_yellow, # asl_pre
    tria_pink  , # asl_model
    tria_aqua  , # asl_post
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
ap.add_argument('-v', '--verbose'    , default=False, action='store_true', help="Enable Verbose mode. Default is off")
ap.add_argument('-w', '--withoutview', default=False, action='store_true', help="Disable Output viewing. Default is on")
ap.add_argument('-z', '--profilelog' , default=False, action='store_true', help="Enable Profile Log (Latency). Default is off")
ap.add_argument('-y', '--profileview', default=False, action='store_true', help="Enable Profile View (Latency). Default is off")
ap.add_argument('-f', '--fps'        , default=False, action='store_true', help="Enable FPS display. Default is off")

args = ap.parse_args()  
  
print('Command line options:')
print(' --input       : ', args.input)
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
    f_profile_csv.write("time,user,hostname,pipeline,detections,resize,detector_pre,detector_model,detector_post,extract_roi,landmark_pre,landmark_model,landmark_post,annotate,asl_pre,asl_model,asl_post,total,fps\n")

pipeline = app_name
detector_type = "blazepalm"
landmark_type = "blazehandlandmark"

#model1 = "blaze_app_python/blaze_tflite/models/palm_detection_lite.tflite"
#blaze_detector = BlazeDetector_tflite(detector_type)
#model1 = "blaze_app_python/blaze_pytorch/models/blazepalm.pth"
#blaze_detector = BlazeDetector_pytorch(detector_type)
model1 = "blaze_app_python/blaze_onnx/models/palm_detection_lite.onnx"
blaze_detector = BlazeDetector_onnx(detector_type)
blaze_detector.set_debug(debug=args.verbose)
blaze_detector.load_model(model1)
 
#model2 = "blaze_app_python/blaze_tflite/models/hand_landmark_lite.tflite"
#blaze_landmark = BlazeLandmark_tflite(landmark_type)
#model2 = "blaze_app_python/blaze_pytorch/models/blazehand_landmark.pth"
#blaze_landmark = BlazeLandmark_pytorch(landmark_type)
model2 = "blaze_app_python/blaze_onnx/models/hand_landmark_lite.onnx"
blaze_landmark = BlazeLandmark_onnx(landmark_type)
blaze_landmark.set_debug(debug=args.verbose)
blaze_landmark.load_model(model2)

thresh_min_score = blaze_detector.min_score_thresh
thresh_min_score_prev = thresh_min_score

thresh_nms = blaze_detector.min_suppression_threshold
thresh_nms_prev = thresh_nms

thresh_confidence = 0.5
thresh_confidence_prev = thresh_confidence

print("================================================================")
print("Hand Controller (ONNX) with ASL (ONNX)")
print("================================================================")
print("\tPress ESC to quit ...")
print("----------------------------------------------------------------")
print("\tPress 'p' to pause video ...")
print("\tPress 'c' to continue ...")
print("\tPress 's' to step one frame at a time ...")
print("\tPress 'w' to take a photo ...")
print("----------------------------------------------------------------")
print("\tPress 'h' to toggle horizontal mirror on input")
print("\tPress 'a' to toggle detection overlay on/off")
print("\tPress 'b' to toggle roi overlay on/off")
print("\tPress 'l' to toggle landmarks overlay on/off")
print("\tPress 'e' to toggle scores image on/off")
print("\tPress 'f' to toggle FPS display on/off")
print("\tPress 'v' to toggle verbose on/off")
print("\tPress 'z' to toggle profiling log on/off")
print("\tPress 'y' to toggle profiling view on/off")
print("----------------------------------------------------------------")
print("\tPress 'n' to toggle use of normalized landmarks for pointnet ...")
print("================================================================")

bStep = False
bPause = False
bWrite = False
bMirrorImage = True
bShowDetection = False
bShowExtractROI = False
bShowLandmarks = True
bShowScores = False
bShowFPS = args.fps
bVerbose = args.verbose
bViewOutput = not args.withoutview
bProfileLog = args.profilelog
bProfileView = args.profileview

bNormalizedLandmarks = True
print("[INFO] Mirror Image = ",bMirrorImage)
print("[INFO] Normalized Landmarks = ",bNormalizedLandmarks)

def ignore(x):
    pass

if bViewOutput:
    app_main_title = "Hand Controller Demo"
    app_ctrl_title = "Hand Controller Demo"
    app_scores_title = "Palm Detection - Detection Scores (sigmoid)"

    cv2.namedWindow(app_main_title)

    cv2.createTrackbar('threshMinScore', app_ctrl_title, int(thresh_min_score*100), 100, ignore)

    cv2.createTrackbar('threshNMS', app_ctrl_title, int(thresh_nms*100), 100, ignore)

    cv2.createTrackbar('threshConfidence', app_ctrl_title, int(thresh_confidence*100), 100, ignore)

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
        
    # Get trackbar values
    if bViewOutput:
        thresh_min_score = cv2.getTrackbarPos('threshMinScore', app_ctrl_title)
        if thresh_min_score < 10:
            thresh_min_score = 10
            cv2.setTrackbarPos('threshMinScore', app_ctrl_title,thresh_min_score)
        thresh_min_score = thresh_min_score*(1/100)
        if thresh_min_score != thresh_min_score_prev:
            blaze_detector.min_score_thresh = thresh_min_score
            thresh_min_score_prev = thresh_min_score
            print("[INFO] thresh_min_score=",thresh_min_score)

        thresh_nms = cv2.getTrackbarPos('threshNMS', app_ctrl_title)
        if thresh_nms > 99:
            thresh_nms = 99
            cv2.setTrackbarPos('threshNMS', app_ctrl_title,thresh_nms)
        thresh_nms = thresh_nms*(1/100)
        if thresh_nms != thresh_nms_prev:
            blaze_detector.min_suppression_threshold = thresh_nms
            thresh_nms_prev = thresh_nms
            print("[INFO] thresh_nms=",thresh_nms)

        thresh_confidence = cv2.getTrackbarPos('threshConfidence', app_ctrl_title)
        thresh_confidence = thresh_confidence*(1/100)
        if thresh_confidence != thresh_confidence_prev:
            thresh_confidence_prev = thresh_confidence
            print("[INFO] thresh_confidence=",thresh_confidence)

    #image = cv2.resize(frame,(0,0), fx=scale, fy=scale) 
    image = frame
    output = image.copy()

    #
    # Profiling
    #

    profile_resize         = 0
    profile_detector_pre   = 0
    profile_detector_model = 0
    profile_detector_post  = 0
    profile_extract_roi    = 0
    profile_landmark_pre   = 0
    profile_landmark_model = 0
    profile_landmark_post  = 0
    profile_annotate       = 0
    profile_asl_pre        = 0
    profile_asl_model      = 0
    profile_asl_post       = 0
    #
    profile_total          = 0
    profile_fps            = 0
    #
    profile_latency_title     = "Latency (sec)"
    profile_performance_title = "Performance (FPS)"


    #            
    # BlazePalm pipeline
    #
    
    start = timer()
    image = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
    img1,scale1,pad1=blaze_detector.resize_pad(image)
    profile_resize = timer()-start

    normalized_detections = blaze_detector.predict_on_image(img1)

    if bShowScores:
        detection_scores_chart = draw_detection_scores( blaze_detector.detection_scores, blaze_detector.min_score_thresh );
        cv2.imshow(app_scores_title,detection_scores_chart);

    if len(normalized_detections) > 0:
  
        start = timer()          
        detections = blaze_detector.denormalize_detections(normalized_detections,scale1,pad1)
                    
        xc,yc,scale,theta = blaze_detector.detection2roi(detections)
        roi_img,roi_affine,roi_box = blaze_landmark.extract_roi(image,xc,yc,theta,scale)
        profile_extract_roi = timer()-start

        results = blaze_landmark.predict(roi_img)
        #flags, normalized_landmarks = results
        flags, normalized_landmarks, handedness_scores = results
                    
        roi_landmarks = normalized_landmarks.copy()
                
        start = timer() 
        landmarks = blaze_landmark.denormalize_landmarks(normalized_landmarks, roi_affine)
        if bShowExtractROI == True:
            draw_roi(output,roi_box)
        if bShowDetection == True:
            draw_detections(output,detections)
        profile_annotate = timer()-start

        #
        # ASL
        # 

        for i in range(len(flags)):
            landmark, flag = landmarks[i], flags[i]

            for i in range(len(flags)):
                flag = flags[i]
                if flag < 0.5:
                   continue

                start = timer()
                landmark = landmarks[i]
                handedness_score = handedness_scores[i]
                roi_landmark = roi_landmarks[i,:,:]
                        
                if bMirrorImage == True:
                    if handedness_score >= 0.5:
                        handedness = "Left"
                    else:
                        handedness = "Right"
                else:                               
                    if handedness_score < 0.5:
                        handedness = "Left"
                    else:
                        handedness = "Right"

                if handedness == "Left":
                    hand_x = 10
                    hand_y = 30
                    #hand_color = (0, 0, 255) # BGR : Red
                    hand_color = (0, 255, 0) # BGR : Green
                    #hand_color = (0, 0, 255) # BGR : Blue
                    hand_msg = 'LEFT='
                else:
                    hand_x = frame_width-128
                    hand_y = 30
                    #hand_color = (0, 0, 255) # BGR : Red
                    #hand_color = (0, 255, 0) # BGR : Green
                    #hand_color = (255, 0, 0) # BGR : Blue
                    hand_color = (190, 161, 0) # aqua (BGR format)
                    hand_msg = 'RIGHT='

                #print(f"Hand[{i}] flag={flag} handedness={handedness} ...")
                #print("Hand[",i,"]")
                #print("    handedness = ",handedness)
                #print("    landmark = ",landmark)
                #print("    roi_landmark = ",roi_landmark)
                        
                # Determine point cloud of hand
                points_raw=[]
                if bNormalizedLandmarks == True:
                    for lm in roi_landmark:
                        points_raw.append([lm[0], lm[1], lm[2]])
                else:                                
                    for lm in landmark:
                        points_raw.append([lm[0], lm[1], lm[2]])
                points_raw = np.array(points_raw)
                #print("    points_raw=",points_raw)

                # Normalize point cloud of hand
                points_norm = points_raw.copy()
                min_x = np.min(points_raw[:, 0])
                max_x = np.max(points_raw[:, 0])
                min_y = np.min(points_raw[:, 1])
                max_y = np.max(points_raw[:, 1])
                for i in range(len(points_raw)):
                    points_norm[i][0] = (points_norm[i][0] - min_x) / (max_x - min_x)
                    points_norm[i][1] = (points_norm[i][1] - min_y) / (max_y - min_y)
                    # PointNet model was trained on left hands, so need to mirror right hand landmarks
                    if bMirrorImage == True and handedness == "Right":
                        points_norm[i][0] = 1.0 - points_norm[i][0]
                    if bMirrorImage == False and handedness == "Left": # for non-mirrored image
                        points_norm[i][0] = 1.0 - points_norm[i][0]
                #print("    points_norm=",points_norm)
                profile_asl_pre += timer()-start

                start = timer()
                # Draw hand landmarks of each hand.
                if bShowLandmarks == True:                
                    draw_landmarks(output, landmark[:,:2], HAND_CONNECTIONS, thickness=2, radius=4, color=hand_color)

                profile_annotate += timer()-start
                        
                start = timer()
                input_name = session_inputs[0].name
                x = np.array([points_norm])
                #print("[INFO] input_name = ",input_name)
                #print("[INFO] x.shape = ",x.shape)
                output_names = [session_outputs[0].name]
                #print("[INFO] output_names = ",output_names)
                result = session.run(output_names, {input_name: x})   
                label = result[0]
                profile_asl_model += timer()-start

                start = timer()
                asl_id = np.argmax(label)
                asl_sign = list(char2int.keys())[list(char2int.values()).index(asl_id)]                
                                    
                #asl_text = '['+str(asl_id)+']='+asl_sign
                asl_text = handedness+"="+asl_sign
                #print(asl_text)
                cv2.putText(output,asl_text,
                    (hand_x,hand_y),
                    text_fontType,text_fontSize,
                    hand_color,text_lineSize,text_lineType)        
                profile_asl_post += timer()-start

    # display real-time FPS counter (if valid)
    if rt_fps_valid == True and bShowFPS:
        cv2.putText(output,rt_fps_message, (rt_fps_x,rt_fps_y),text_fontType,text_fontSize,text_color,text_lineSize,text_lineType)
        if not bViewOutput:
            print("[REAL-TIME]",rt_fps_message)

    #
    # Profiling
    #
    profile_detector_pre   = blaze_detector.profile_pre
    profile_detector_model = blaze_detector.profile_model
    profile_detector_post  = blaze_detector.profile_post
    profile_detector = profile_detector_pre + profile_detector_model + profile_detector_post
    if len(normalized_detections) > 0:
        profile_landmark_pre   = blaze_landmark.profile_pre
        profile_landmark_model = blaze_landmark.profile_model
        profile_landmark_post  = blaze_landmark.profile_post
    profile_landmark = profile_landmark_pre + profile_landmark_model + profile_landmark_post
    profile_asl = profile_asl_pre + profile_asl_model + profile_asl_post
    profile_total = profile_resize + \
                    profile_detector + \
                    profile_extract_roi + \
                    profile_landmark + \
                    profile_annotate + \
                    profile_asl
    profile_fps = 1.0 / profile_total

    if bProfileLog == True:
        # display profiling results to console
        print(f"[PROFILING] hands={len(normalized_detections)}, FPS={profile_fps:.3f}fps, Total={profile_total*1000:.3f}ms, Detection={profile_detector*1000:.3f}ms, Extract={profile_extract_roi*1000:.3f}ms, Landmark={profile_landmark*1000:.3f}ms, Annotate={profile_annotate*1000:.3f}ms, ASL={profile_asl*1000:.3f}ms")
        # write profiling results to csv file
        timestamp = datetime.now()
        csv_str = \
            str(timestamp)+","+\
            str(user)+","+\
            str(host)+","+\
            pipeline+","+\
            str(len(normalized_detections))+","+\
            str(profile_resize)+","+\
            str(profile_detector_pre)+","+\
            str(profile_detector_model)+","+\
            str(profile_detector_post)+","+\
            str(profile_extract_roi)+","+\
            str(profile_landmark_pre)+","+\
            str(profile_landmark_model)+","+\
            str(profile_landmark_post)+","+\
            str(profile_annotate)+","+\
            str(profile_asl_pre)+","+\
            str(profile_asl_model)+","+\
            str(profile_asl_post)+","+\
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
        

    if bProfileView:
        #
        # Latency
        #
        component_labels = [
            "resize",
            "detector[pre]",
            "detector[model]",
            "detector[post]",
            "extract_roi",
            "landmark[pre]",
            "landmark[model]",
            "landmark[post]",
            "annotate",
            "asl[pre]",
            "asl[model]",
            "asl[post]"
        ]
        pipeline_titles = [app_name]
        component_values=[
            [profile_resize],
            [profile_detector_pre],
            [profile_detector_model],
            [profile_detector_post],
            [profile_extract_roi],
            [profile_landmark_pre],
            [profile_landmark_model],
            [profile_landmark_post],
            [profile_annotate],
            [profile_asl_pre],
            [profile_asl_model],
            [profile_asl_post]
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

    if key == 97: # 'a'
        bShowDetection = not bShowDetection  
        print("[INFO] bShowDetection=",bShowDetection)

    if key == 98: # 'b'
        bShowExtractROI = not bShowExtractROI  
        print("[INFO] bShowExtractROI=",bShowExtractROI)

    if key == 108: # 'l'
        bShowLandmarks = not bShowLandmarks
        print("[INFO] bShowLandmarks=",bShowLandmarks)             
                
    if key == 101: # 'e'
        bShowScores = not bShowScores
        print("[INFO] bShowScores=",bShowScores)
        if not bShowScores:
           cv2.destroyWindow(app_scores_title)

    if key == 102: # 'f'
        bShowFPS = not bShowFPS
        print("[INFO] bShowFPS=",bShowFPS)

    if key == 118: # 'v'
        bVerbose = not bVerbose
        blaze_detector.set_debug(debug=bVerbose) 
        blaze_landmark.set_debug(debug=bVerbose)
        print("[INFO] Verbose = ",bVerbose)

    if key == 122: # 'z'
        bProfileLog = not bProfileLog
        print("[INFO] bProfileLog=",bProfileLog)

    if key == 121: # 'y'
        bProfileView = not bProfileView 
        print("[INFO] bProfileView=",bProfileView)
        blaze_detector.set_profile(profile=bProfileView) 
        blaze_landmark.set_profile(profile=bProfileView)
        if not bProfileView:
            cv2.destroyWindow(profile_latency_title)
            cv2.destroyWindow(profile_performance_title)

    if key == 110: # 'n'
        bNormalizedLandmarks = not bNormalizedLandmarks        
        print("[INFO] bNormalizedLandmarks=",bNormalizedLandmarks)
    
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

# Cleanup
f_profile_csv.close()
cv2.destroyAllWindows()
