# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import numpy as np
import sys
import os
import importlib

from dataclasses import dataclass

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
# import CV BRIDGE
from cv_bridge import CvBridge, CvBridgeError
import cv2

from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PoseStamped, TransformStamped

#
# Tria color palette
#

# Primary Palette (RGB format)
tria_blue   = (  0,  31,  99); # TRIA BLUE
tria_pink   = (255,   0, 163); # TRIA PINK
tria_white  = (255, 255, 255); # WHITE

# Secondary Palette (RGB format)
tria_gray11 = ( 83,  86,  90); # COOL GRAY 11
tria_gray7  = (151, 153, 155); # COOL GRAY 7
tria_gray3  = (200, 201, 199); # COOL GRAY 3

# Tertiary Palette (RGB format)
tria_purple = (107,  83, 157); # TRIA PURPLE
tria_yellow = (235, 201,  80); # TRIA YELLOW
tria_aqua   = (  0, 161, 190); # TRIA AQUA
tria_black  = (  0,   0,   0); # BLACK

# Visual Control Dials

#CV_DRAW_COLOR_PRIMARY = (255, 255, 0)
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

class HandControllerMp1DialsPoseNode(Node):

    def __init__(self):
        super().__init__('hand_controller_mp1dials_pose_node')
        self.subscriber1_ = self.create_subscription(Image,'image_raw',self.listener_callback,10)
        self.subscriber1_  # prevent unused variable warning
        self.publisher1_ = self.create_publisher(Image, 'hand_controller/image_annotated', 10)
        # Create publishers for the current and target pose
        self.publisher3_ = self.create_publisher(PoseStamped, 'hand_controller/current_pose', 10)        
        self.publisher4_ = self.create_publisher(PoseStamped, 'hand_controller/target_pose', 10)        

        # verbose
        self.declare_parameter("verbose", True)
        self.verbose = self.get_parameter('verbose').value          
        self.get_logger().info('Verbose : "%s"' % self.verbose)

        # use_imshow
        self.declare_parameter("use_imshow", True)
        self.use_imshow = self.get_parameter('use_imshow').value          
        self.get_logger().info('Use imshow : "%s"' % self.use_imshow)

        # Repo Path
        self.declare_parameter("repo_path", "/root/hand_controller")
        self.repo_path = self.get_parameter('repo_path').value
        self.get_logger().info('Repo path : "%s"' % self.repo_path)

        # Additional Settings (could eventually be mapped to parameters)
        self.bMirrorImage = True
        self.bNormalizedLandmarks = True
        self.bShowDetection = False
        self.bShowLandmarks = True
        
        # Blaze models
        self.declare_parameter("blaze_target", "blaze_tflite")
        self.declare_parameter("blaze_model1", "palm_detection_lite.tflite")
        self.declare_parameter("blaze_model2", "hand_landmark_lite.tflite")
        self.blaze_target = self.get_parameter('blaze_target').value
        self.blaze_model1 = self.get_parameter('blaze_model1').value
        self.blaze_model2 = self.get_parameter('blaze_model2').value
        sys.path.append(os.path.join(self.repo_path,"blaze_app_python"))
        sys.path.append(os.path.join(self.repo_path,"blaze_app_python","blaze_common"))
        sys.path.append(os.path.join(self.repo_path,"blaze_app_python",self.blaze_target))
        #BlazeDetector = importlib.import_module(self.blaze_target+".blazedetector")
        #BlazeLandmark = importlib.import_module(self.blaze_target+".blazelandmark")
        if self.blaze_target == "blaze_tflite":
            from blaze_tflite.blazedetector import BlazeDetector
            from blaze_tflite.blazelandmark import BlazeLandmark
        if self.blaze_target == "blaze_pytorch":
            from blaze_pytorch.blazedetector import BlazeDetector
            from blaze_pytorch.blazelandmark import BlazeLandmark
        #
        self.detector_type = "blazepalm"
        self.landmark_type = "blazehandlandmark"
        #
        self.blaze_model1_fullpath = os.path.join(self.repo_path,"blaze_app_python",self.blaze_target,"models",self.blaze_model1)
        self.get_logger().info('Blaze Detector model : "%s"' % self.blaze_model1_fullpath)
        self.blaze_detector = BlazeDetector(self.detector_type)
        self.blaze_detector.set_debug(debug=self.verbose)
        self.blaze_detector.load_model(self.blaze_model1_fullpath)
        self.blaze_detector.min_score_thresh = 0.8 # increase threshold to prevent false positives
        #
        self.blaze_model2_fullpath = os.path.join(self.repo_path,"blaze_app_python",self.blaze_target,"models",self.blaze_model2)
        self.get_logger().info('Blaze Landmark model : "%s"' % self.blaze_model2_fullpath)
        self.blaze_landmark = BlazeLandmark(self.landmark_type)
        self.blaze_landmark.set_debug(debug=self.verbose)
        self.blaze_landmark.load_model(self.blaze_model2_fullpath)

        # Visual Control Dials

        # Declare frames as parameters (can be overridden at launch)
        self.declare_parameter('source_frame', 'base_link')
        self.declare_parameter('target_frame', 'gripper_base')

        self.source_frame = self.get_parameter('source_frame').get_parameter_value().string_value
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value

        # TF2 setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Additional Settings (for text overlay)
        self.scale = 1.0
        self.text_fontType = cv2.FONT_HERSHEY_SIMPLEX
        self.text_fontSize = 0.75*self.scale
        self.text_color    = (255,0,0)
        self.text_lineSize = max( 1, int(2*self.scale) )
        self.text_lineType = cv2.LINE_AA
        self.text_x = int(10*self.scale)
        self.text_y = int(30*self.scale)        

        self.get_logger().info("Initialization Successful")


    def listener_callback(self, msg):
        bridge = CvBridge()
        cv2_image = bridge.imgmsg_to_cv2(msg,desired_encoding = "rgb8")
        
        # Mirror horizontally for selfie-mode
        cv2_image = cv2.flip(cv2_image, 1)

        # Process with blaze models.
        image = cv2_image
        image_height, image_width, _ = image.shape
        annotated_image = image.copy()

        #            
        # Visual Control Dials (init)
        #

        lh_data, rh_data = None, None        

        #            
        # BlazePalm pipeline
        #

        from visualization import draw_detections
        #from visualization import draw_roi, draw_landmarks
        #from visualization import HAND_CONNECTIONS
    
        #image = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
        img1,scale1,pad1 = self.blaze_detector.resize_pad(image)

        normalized_detections = self.blaze_detector.predict_on_image(img1)
        if len(normalized_detections) > 0:
  
            detections = self.blaze_detector.denormalize_detections(normalized_detections,scale1,pad1)
                    
            xc,yc,scale,theta = self.blaze_detector.detection2roi(detections)
            #roi_img,roi_affine,roi_box = self.blaze_landmark.extract_roi(image,xc,yc,theta,scale)

            #results = self.blaze_landmark.predict(roi_img)
            #flags, normalized_landmarks, handedness_scores = results
                    
            #roi_landmarks = normalized_landmarks.copy()
                
            #landmarks = self.blaze_landmark.denormalize_landmarks(normalized_landmarks, roi_affine)
            if self.bShowDetection == True:
            #    draw_roi(annotated_image,roi_box)
                draw_detections(annotated_image,detections)

            #
            # Visual Control Dials (Hand Data)
            # 

            for i in range(len(normalized_detections)):
                hand_xc = xc[i]
                hand_yc = yc[i]
                hand_z  = 0.0

                landmarks = np.asarray([[hand_xc,hand_yc,hand_z]])
            
                # Visual Control Dials (prepare hand data)
                if hand_xc < (image_width/2):
                    handedness = "Left"
                    lh_data = HandData(handedness, landmarks, image_width, image_height)                
                else:
                    handedness = "Right"
                    rh_data = HandData(handedness, landmarks, image_width, image_height)

        # Visual Control Dials (display dials)
        if lh_data:
            cv2.circle(annotated_image, (int(lh_data.center_perc[0]*image_width), int(lh_data.center_perc[1]*image_height)), radius=10, color=tria_pink, thickness=-1)  
        if rh_data:
            cv2.circle(annotated_image, (int(rh_data.center_perc[0]*image_width), int(rh_data.center_perc[1]*image_height)), radius=10, color=tria_pink, thickness=-1)
        delta_xy, delta_z = draw_control_overlay(annotated_image, lh_data, rh_data)
        #self.get_logger().info(f"delta_xy = {delta_xy} | delta_z = {delta_z}")
   
        if delta_xy[1] != 0 or delta_xy[0] != 0 or delta_z[1] != 0:
            try:
                now = rclpy.time.Time()
                trans: TransformStamped = self.tf_buffer.lookup_transform(
                    self.source_frame,
                    self.target_frame,
                    now,
                    timeout=rclpy.duration.Duration(seconds=0.5)
                )

                current_pose_msg = PoseStamped()
                current_pose_msg.header = trans.header
                current_pose_msg.pose.position.x = trans.transform.translation.x
                current_pose_msg.pose.position.y = trans.transform.translation.y
                current_pose_msg.pose.position.z = trans.transform.translation.z
                current_pose_msg.pose.orientation = trans.transform.rotation

                self.publisher3_.publish(current_pose_msg)

                if self.verbose:
                    self.get_logger().info(f"Published current pose: {current_pose_msg.pose.position:+.3f}")
                        
                target_pose_msg = PoseStamped()
                target_pose_msg.header = trans.header
                target_pose_msg.pose.position.x = trans.transform.translation.x
                target_pose_msg.pose.position.y = trans.transform.translation.y
                target_pose_msg.pose.position.z = trans.transform.translation.z
                target_pose_msg.pose.orientation = trans.transform.rotation

                # Left/Right                        
                target_pose_msg.pose.position.x += delta_xy[0] * 0.2

                 # Advance/BackUp
                target_pose_msg.pose.position.y += delta_xy[1] * 0.2

                # Up/Down
                target_pose_msg.pose.position.z += delta_z[1] * 0.2

                self.publisher4_.publish(target_pose_msg)
                if self.verbose:
                    self.get_logger().info(f"Published target  pose: {target_pose_msg.pose.position:+.3f}")
                        
            except Exception as e:
                self.get_logger().warn(f"TF lookup failed: {e}")

        if self.use_imshow == True:
            # DISPLAY
            cv2_bgr_image = cv2.cvtColor(annotated_image, cv2.COLOR_RGB2BGR)
            cv2.imshow('hand_controller_mp1dials_pose_node',cv2_bgr_image)
            cv2.waitKey(1)                    
        
        # CONVERT BACK TO ROS & PUBLISH
        image_ros = bridge.cv2_to_imgmsg(annotated_image, encoding="rgb8")        
        self.publisher1_.publish(image_ros)


def main(args=None):
    rclpy.init(args=args)

    hand_controller_mp1dials_pose_node = HandControllerMp1DialsPoseNode()

    rclpy.spin(hand_controller_mp1dials_pose_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    hand_controller_mp1dials_pose_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
