# Updated usbcam_subscriber_flask_node.py with better debugging

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from flask import Flask, Response, render_template_string, render_template, jsonify
import cv2
import threading
import time
import os
import socket
import json
import numpy as np

app = Flask(__name__,
    template_folder='/root/hand_controller/templates',
    static_folder='/root/hand_controller/static')
bridge = CvBridge()
latest_frame = None
latest_hand_data = None
latest_frame_lock = threading.Lock()
latest_hand_data_lock = threading.Lock()

# Enable template auto-reload
app.config['TEMPLATES_AUTO_RELOAD'] = True
app.config['SEND_FILE_MAX_AGE_DEFAULT'] = 0  # Disable caching

# Global variable for Flask to access
FLASK_ARGS = {}

class UsbCamSubscriberFlaskNode(Node):
    def __init__(self):
        super().__init__('usbcam_subscriber_flask_node')

        self.declare_parameter('model', 'default_model.eim')
        self.declare_parameter('verbose', True)  # Set to True for debugging
        self.declare_parameter('use_imshow', False)
        self.declare_parameter('hand_controller_topic', 'hand_controller/cmd_vel')
        
        # Get parameter values
        self.model_path = self.get_parameter('model').value
        self.verbose = self.get_parameter('verbose').value
        self.use_imshow = self.get_parameter('use_imshow').value
        hand_topic = self.get_parameter('hand_controller_topic').value

        self.get_logger().info(f'Model path: {self.model_path}')
        self.get_logger().info(f'Hand controller topic: {hand_topic}')
        self.get_logger().info(f'Verbose mode: {self.verbose}')

        global FLASK_ARGS
        FLASK_ARGS = {
            'model': self.model_path,
            'verbose': self.verbose,
            'use_imshow': self.use_imshow,
            'hostname': socket.gethostname(),
            'hand_topic': hand_topic
        }

        # Subscribe to camera feed
        self.image_subscription = self.create_subscription(
            Image,
            'image_raw',
            self.process_video,
            10)
        self.get_logger().info('Subscribed to image_raw topic')
        
        # Subscribe to hand controller twist data - try both topics
        self.hand_subscription = self.create_subscription(
            Twist,
            hand_topic,
            self.process_hand_data,
            10)
        self.get_logger().info(f'Subscribed to {hand_topic} topic')
        
        # Also subscribe to /cmd_vel as backup
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.process_hand_data,
            10)
        self.get_logger().info('Subscribed to /cmd_vel topic as backup')
        
        # Timer to check for topic activity
        self.last_message_time = None
        self.message_count = 0
        self.check_timer = self.create_timer(2.0, self.check_topic_activity)
        
        self.get_logger().info('USB Camera Subscriber Flask Node started')

    def check_topic_activity(self):
        """Check if we're receiving messages and log if not"""
        if self.last_message_time is None:
            self.get_logger().warn('No hand controller messages received yet')
            
            # Check what's actually being published
            topic_list = self.get_topic_names_and_types()
            active_topics = []
            for topic_name, topic_types in topic_list:
                publishers = self.count_publishers(topic_name)
                if publishers > 0:
                    active_topics.append((topic_name, publishers))
            
            if active_topics:
                self.get_logger().info(f'Active topics with publishers: {active_topics}')
            else:
                self.get_logger().warn('No topics have active publishers!')
        else:
            time_since_last = time.time() - self.last_message_time
            if time_since_last > 5.0:
                self.get_logger().warn(f'No hand controller messages for {time_since_last:.1f} seconds')

    def process_video(self, image_msg):
        global latest_frame
        try:
            cv_image = bridge.imgmsg_to_cv2(image_msg, "bgr8")
            ret, jpeg = cv2.imencode('.jpg', cv_image)
            with latest_frame_lock:
                latest_frame = jpeg.tobytes()
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")

    def process_hand_data(self, twist_msg):
        global latest_hand_data
        try:
            self.last_message_time = time.time()
            self.message_count += 1
            
            # Extract control data from twist message
            linear_x = twist_msg.linear.x    # Forward/Backward (Right hand)
            angular_z = twist_msg.angular.z  # Left/Right turning (Left hand)
            
            # Log first few messages for debugging
            if self.message_count <= 10:
                self.get_logger().info(f'Message {self.message_count}: linear.x={linear_x:.3f}, angular.z={angular_z:.3f}')
            elif self.message_count == 11:
                self.get_logger().info('... continuing to receive messages (suppressing logs)')
            
            # Calculate normalized values - SEPARATE CONTROLS
            # Left hand: only angular_z (turning)
            normalized_angular = max(min(angular_z / 2.0, 1.0), -1.0) if abs(angular_z) > 0.1 else 0.0
            
            # Right hand: only linear_x (forward/backward)
            normalized_linear = max(min(linear_x / 2.0, 1.0), -1.0) if abs(linear_x) > 0.1 else 0.0
            
            # Determine hand activity based on SEPARATE controls
            left_hand_active = abs(normalized_angular) > 0.1  # Only angular movement
            right_hand_active = abs(normalized_linear) > 0.1  # Only linear movement
            
            # Calculate visual positions based on SEPARATE controls
            # Left hand: only moves left/right based on angular control
            lh_screen_x = 0.5 + (normalized_angular * 0.4)  # Map to 0.1-0.9 range
            lh_screen_y = 0.5  # Fixed vertical position - doesn't move up/down
            
            # Right hand: only moves up/down based on linear control  
            rh_screen_x = 0.5  # Fixed horizontal position - doesn't move left/right
            rh_screen_y = 0.5 + (normalized_linear * 0.4)   # Map to 0.1-0.9 range
            
            hand_data = {
                'left_hand': {
                    'x_position': normalized_angular,  # Turning control
                    'y_position': 0.0,                 # No vertical movement for left hand
                    'screen_x': max(min(lh_screen_x, 0.9), 0.1),
                    'screen_y': max(min(lh_screen_y, 0.9), 0.1),  # Fixed at 0.5
                    'active': left_hand_active
                },
                'right_hand': {
                    'z_position': normalized_linear,   # Forward/backward control
                    'screen_x': max(min(rh_screen_x, 0.9), 0.1),  # Fixed at 0.5
                    'screen_y': max(min(rh_screen_y, 0.9), 0.1),
                    'active': right_hand_active
                },
                'timestamp': time.time(),
                'raw_data': {
                    'linear_x': linear_x,
                    'angular_z': angular_z
                },
                'status': 'active',
                'message_count': self.message_count
            }
            
            with latest_hand_data_lock:
                latest_hand_data = hand_data
                
        except Exception as e:
            self.get_logger().error(f"Error processing hand data: {e}")

def generate_frames():
    global latest_frame
    while True:
        with latest_frame_lock:
            if latest_frame is not None:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + latest_frame + b'\r\n')
        time.sleep(0.03)

@app.route("/")
def index():
    model_name = os.path.basename(FLASK_ARGS.get('model', 'unknown_model.eim'))
    verbose = FLASK_ARGS.get('verbose', False)
    use_imshow = FLASK_ARGS.get('use_imshow', False)
    hostname = FLASK_ARGS.get('hostname', False)
    return render_template('index.html',
        model_name=model_name,
        verbose=verbose,
        use_imshow=use_imshow,
        hostname=hostname
    )

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/hand_data')
def hand_data():
    global latest_hand_data
    try:
        with latest_hand_data_lock:
            if latest_hand_data is not None:
                return jsonify(latest_hand_data)
            else:
                return jsonify({
                    'left_hand': {
                        'x_position': 0, 
                        'y_position': 0, 
                        'screen_x': 0.5,
                        'screen_y': 0.5,
                        'active': False
                    },
                    'right_hand': {
                        'z_position': 0,
                        'screen_x': 0.5,
                        'screen_y': 0.5,
                        'active': False
                    },
                    'timestamp': time.time(),
                    'raw_data': {'linear_x': 0, 'angular_z': 0},
                    'status': 'no_data',
                    'message_count': 0
                })
    except Exception as e:
        return jsonify({'error': str(e), 'status': 'error'}), 500

@app.route('/debug')
def debug():
    """Debug endpoint to check what's happening"""
    global latest_hand_data
    with latest_hand_data_lock:
        data_status = 'has data' if latest_hand_data is not None else 'no data'
    
    return jsonify({
        'flask_status': 'running',
        'hand_data_status': data_status,
        'timestamp': time.time(),
        'parameters': FLASK_ARGS
    })

@app.route('/health')
def health():
    return {'status': 'healthy', 'timestamp': time.time()}

def ros2_thread():
    rclpy.init()
    node = UsbCamSubscriberFlaskNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

def main():    
    ros_thread = threading.Thread(target=ros2_thread)
    ros_thread.daemon = True
    ros_thread.start()
    
    # Run with debug mode for auto-reload
    if os.environ.get('FLASK_DEBUG'):
        app.run(host='0.0.0.0', port=5001, debug=True, use_reloader=True, threaded=True)
    else:
        app.run(host='0.0.0.0', port=5001, debug=False, threaded=True)

if __name__ == '__main__':
    main()