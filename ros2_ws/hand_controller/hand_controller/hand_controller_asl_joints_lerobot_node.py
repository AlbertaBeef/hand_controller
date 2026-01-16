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

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
# import CV BRIDGE
from cv_bridge import CvBridge, CvBridgeError
import cv2

#from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# PointNet (for Hands) references : 
#    https://medium.com/@er_95882/asl-recognition-using-pointnet-and-mediapipe-f2efda78d089
#    https://www.kaggle.com/datasets/ayuraj/asl-dataset
#    https://github.com/e-roe/pointnet_hands/tree/main
import torch

#sys.path.append('/media/albertabeef/Tycho/asl_mediapipe_pointnet/model')
#from point_net import PointNet

device = 'cuda' if torch.cuda.is_available() else 'cpu'

#char2int = {
#            "a":0, "b":1, "c":2, "d":3, "e":4, "f":5, "g":6, "h":7, "i":8, "k":9, "l":10, "m":11,
#            "n":12, "o":13, "p":14, "q":15, "r":16, "s":17, "t":18, "u":19, "v":20, "w":21, "x":22, "y":23
#            }
char2int = {
            "A":0, "B":1, "C":2, "D":3, "E":4, "F":5, "G":6, "H":7, "I":8, "K":9, "L":10, "M":11,
            "N":12, "O":13, "P":14, "Q":15, "R":16, "S":17, "T":18, "U":19, "V":20, "W":21, "X":22, "Y":23
            }


@torch.no_grad()        
class HandControllerAslJointsNode(Node):

    def __init__(self):
        super().__init__('hand_controller_asl_joints_lerobot_node')
        self.subscriber1_ = self.create_subscription(Image,'image_raw',self.listener_callback,10)
        self.subscriber1_  # prevent unused variable warning
        self.publisher1_ = self.create_publisher(Image, 'hand_controller/image_annotated', 10)
        # Create publishers for the '/arm_controller/joint_trajectory' topic
        self.publisher2_ = self.create_publisher(JointState, '/so101_follower/joint_commands', 10)

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
        #
        self.blaze_model2_fullpath = os.path.join(self.repo_path,"blaze_app_python",self.blaze_target,"models",self.blaze_model2)
        self.get_logger().info('Blaze Landmark model : "%s"' % self.blaze_model2_fullpath)
        self.blaze_landmark = BlazeLandmark(self.landmark_type)
        self.blaze_landmark.set_debug(debug=self.verbose)
        self.blaze_landmark.load_model(self.blaze_model2_fullpath)

        # ASL model (pointnet)
        self.asl_model_path = os.path.join(self.repo_path,"asl_pointnet")
        self.asl_model_name = "point_net_1.pth"
        sys.path.append(self.asl_model_path)
        self.asl_model_fullpath = os.path.join(self.asl_model_path,self.asl_model_name)
        self.get_logger().info('ASL model : "%s"' % self.asl_model_fullpath)
        self.asl_model = torch.load(self.asl_model_fullpath,weights_only=False,map_location=device)
        #self.asl_model.eval() # set dropout and batch normalization layers to evaluation mode before running inference

        # Sign Detection status
        self.asl_sign = ""
        self.actionDetected = ""        

        # Create the JointTrajectory messages
        self.lerobot_joint_states = JointState()
        self.lerobot_joint_states.name     = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']
        self.lerobot_joint_states.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.lerobot_joint_states.velocity = []
        self.lerobot_joint_states.effort   = []                
        
        # Publish the message
        if self.verbose:
            #self.get_logger().info(f"Publishing arm joint angles : {self.arm_trajectory_command.points}")
            shoulder_pan_joint = self.lerobot_joint_states.position[0]
            shoulder_lift_joint = self.lerobot_joint_states.position[1]
            elbow_joint = self.lerobot_joint_states.position[2]
            self.get_logger().info(f"ShoulderPanJoint={shoulder_pan_joint:+.3f} ShoulderLiftJoint={shoulder_lift_joint:+.3f} ElbowJoint={elbow_joint:+.3f}")
        self.publisher2_.publish(self.lerobot_joint_states)

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
        # BlazePalm pipeline
        #

        from visualization import draw_detections
        from visualization import draw_roi, draw_landmarks
        from visualization import HAND_CONNECTIONS
    
        #image = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
        img1,scale1,pad1 = self.blaze_detector.resize_pad(image)

        normalized_detections = self.blaze_detector.predict_on_image(img1)
        if len(normalized_detections) > 0:
  
            detections = self.blaze_detector.denormalize_detections(normalized_detections,scale1,pad1)
                    
            xc,yc,scale,theta = self.blaze_detector.detection2roi(detections)
            roi_img,roi_affine,roi_box = self.blaze_landmark.extract_roi(image,xc,yc,theta,scale)

            results = self.blaze_landmark.predict(roi_img)
            #flags, normalized_landmarks = results
            flags, normalized_landmarks, handedness_scores = results
                    
            roi_landmarks = normalized_landmarks.copy()
                
            landmarks = self.blaze_landmark.denormalize_landmarks(normalized_landmarks, roi_affine)
            if self.bShowDetection == True:
                draw_roi(annotated_image,roi_box)
                draw_detections(annotated_image,detections)

            #
            # ASL
            # 

            for i in range(len(flags)):

                flag = flags[i]
                if flag < 0.5:
                   continue

                landmark = landmarks[i]
                handedness_score = handedness_scores[i]
                roi_landmark = roi_landmarks[i,:,:]
                        
                if self.bMirrorImage == True:
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
                    hand_color = (0, 255, 0) # RGB : Green
                    hand_msg = 'LEFT='
                else:
                    hand_x = image_width-256
                    hand_y = 30
                    hand_color = (255, 0, 0) # RGB : Red
                    hand_msg = 'RIGHT='
                        
                # Determine point cloud of hand
                points_raw=[]
                if self.bNormalizedLandmarks == True:
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
                    if self.bMirrorImage == True and handedness == "Right":
                        points_norm[i][0] = 1.0 - points_norm[i][0]
                    if self.bMirrorImage == False and handedness == "Left": # for non-mirrored image
                        points_norm[i][0] = 1.0 - points_norm[i][0]
                                                                
                # Draw hand landmarks of each hand.
                if self.bShowLandmarks == True:                
                    draw_landmarks(annotated_image, landmark[:,:2], HAND_CONNECTIONS, thickness=2, radius=4, color=hand_color)
	
                asl_sign = ""
                self.actionDetected = ""
                try:
                    pointst = torch.tensor([points_norm]).float().to(device)
                    label = self.asl_model(pointst)
                    label = label.detach().cpu().numpy()
                    #self.get_logger().info('Detected Labels: "%s"' % label)                    
                    asl_id = np.argmax(label)
                    asl_sign = list(char2int.keys())[list(char2int.values()).index(asl_id)]                
                    #self.get_logger().info('Detected Sign: "%s"' % asl_sign)

                    asl_text = hand_msg+asl_sign
                    cv2.putText(annotated_image,asl_text,
                        (hand_x,hand_y),
                        self.text_fontType,self.text_fontSize,
                        hand_color,self.text_lineSize,self.text_lineType)

                    if handedness == "Left":
                        # Define action
                        if asl_sign == 'A':
                          self.actionDetected = "A : Advance"
                        if asl_sign == 'B':
                          self.actionDetected = "B : Back-Up"
                        if asl_sign == 'L':
                          self.actionDetected = "L : Left"
                        if asl_sign == 'R':
                          self.actionDetected = "R : Right"
                        if asl_sign == 'U':
                          self.actionDetected = "U : Up"
                        if asl_sign == 'Y':
                          self.actionDetected = "Y : Down"

                        action_text = '['+self.actionDetected+']'
                        cv2.putText(annotated_image,action_text,
                            (hand_x,hand_y*2),
                            self.text_fontType,self.text_fontSize,
                            hand_color,self.text_lineSize,self.text_lineType)

                        if self.verbose:
                            self.get_logger().info(f"{asl_text} => {action_text}")

 
                    if handedness == "Right":
                        # Define action
                        if asl_sign == 'A':
                          self.actionDetected = "A : Close Gripper"
                        if asl_sign == 'B':
                          self.actionDetected = "B : Open Gripper"

                        action_text = '['+self.actionDetected+']'
                        cv2.putText(annotated_image,action_text,
                            (hand_x,hand_y*2),
                            self.text_fontType,self.text_fontSize,
                            hand_color,self.text_lineSize,self.text_lineType)

                        if self.verbose:
                            self.get_logger().info(f"{asl_text} => {action_text}")


                except:
                    #print("[ERROR] Exception occured during ASL classification ...")
                    self.get_logger().warning('Exception occured during ASL Classification ...') 

                if handedness == "Left" and self.actionDetected != "":
                    try:
                        # shoulder pan joint : index 0, range -2.055(L) to +2.055(R)
                        shoulder_pan_joint = self.lerobot_joint_states.position[0]
                        if self.actionDetected == "L : Left":
                            shoulder_pan_joint -= 0.01
                            if shoulder_pan_joint < -2.05:
                                shoulder_pan_joint = -2.05
                        if self.actionDetected == "R : Right":
                            shoulder_pan_joint += 0.01
                            if shoulder_pan_joint > +2.05:
                                shoulder_pan_joint = +2.05
                        self.lerobot_joint_states.position[0] = shoulder_pan_joint
                                                
                        # shoulder lift joint : index 1, range +2.01(A) to -2.01(B)
                        shoulder_lift_joint = self.lerobot_joint_states.position[1]
                        if self.actionDetected == "A : Advance":
                            shoulder_lift_joint += 0.01
                            if shoulder_lift_joint > +2.01:
                                shoulder_lift_joint = +2.01
                        if self.actionDetected == "B : Back-Up":
                            shoulder_lift_joint -= 0.01
                            if shoulder_lift_joint < -2.01:
                                shoulder_lift_joint = -2.01
                        self.lerobot_joint_states.position[1] = shoulder_lift_joint

                        # wrist_flex : index 3, range -1.78(U) to +1.78(D)
                        wrist_flex = self.lerobot_joint_states.position[3]
                        if self.actionDetected == "U : Up":
                            wrist_flex -= 0.01
                            if wrist_flex < -1.78:
                                wrist_flex = -1.78
                        if self.actionDetected == "Y : Down":
                            wrist_flex += 0.01
                            if wrist_flex > +1.78:
                                wrist_flex = +1.78
                        self.lerobot_joint_states.position[3] = wrist_flex
                        

                        if self.verbose:
                            self.get_logger().info(f"shoulder_pan={shoulder_pan_joint:+.3f} shoulder_lift={shoulder_lift_joint:+.3f} wrist_flex={wrist_flex:+.3f}")

                        # Publish the message
                        self.publisher2_.publish(self.lerobot_joint_states)
                        

                    except Exception as e:
                        self.get_logger().warn(f"Error publishing arm joint angles: {e}")

                if handedness == "Right" and self.actionDetected != "":
                    try:
                        # gripper : index 5, range -1.75(closed) - +1.75(open)
                        gripper = self.lerobot_joint_states.position[5]
                        if self.actionDetected == "A : Close Gripper":
                            gripper -= 0.05
                            if gripper < -1.75:
                                gripper = -1.75
                        if self.actionDetected == "B : Open Gripper":
                            gripper += 0.05
                            if gripper > +1.75:
                                gripper = +1.75
                        self.lerobot_joint_states.position[5] = gripper


                        if self.verbose:
                            self.get_logger().info(f"gripper={gripper:+.3f}")

                        # Publish the message
                        self.publisher2_.publish(self.lerobot_joint_states)

                    except Exception as e:
                        self.get_logger().warn(f"Error publishing gripper joint angles: {e}")
                
        if self.use_imshow == True:
            # DISPLAY
            cv2_bgr_image = cv2.cvtColor(annotated_image, cv2.COLOR_RGB2BGR)
            cv2.imshow('hand_controller_asl_joints_lerobot_node',cv2_bgr_image)
            cv2.waitKey(1)                    
        
        # CONVERT BACK TO ROS & PUBLISH
        image_ros = bridge.cv2_to_imgmsg(annotated_image, encoding="rgb8")        
        self.publisher1_.publish(image_ros)


def main(args=None):
    rclpy.init(args=args)

    hand_controller_asl_joints_node = HandControllerAslJointsNode()

    rclpy.spin(hand_controller_asl_joints_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    hand_controller_asl_joints_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
