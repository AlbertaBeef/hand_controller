# gtk_gui_node.py

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String
from sensor_msgs.msg import Image as RosImage
from cv_bridge import CvBridge
import cv2
import threading
from collections import deque
import time

import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, GObject, GLib, GdkPixbuf

# --- 1. The Worker Thread: For Heavy Lifting ---
class FrameProcessor(threading.Thread):
    """A dedicated worker thread for converting OpenCV frames to GdkPixbufs."""
    def __init__(self, input_queue, output_queue, logger):
        super().__init__()
        self.input_queue = input_queue
        self.output_queue = output_queue
        self.logger = logger
        self.daemon = True  # Allows main program to exit even if this thread is running
        self._is_running = threading.Event()
        self._is_running.set()

    def run(self):
        """The main loop of the worker thread."""
        while self._is_running.is_set():
            try:
                # Block until a frame is available or timeout
                cv_image = self.input_queue.popleft()
                
                # --- The expensive work happens here ---
                h, w, c = cv_image.shape
                rgb_frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
                pixbuf = GdkPixbuf.Pixbuf.new_from_data(
                    rgb_frame.tobytes(), GdkPixbuf.Colorspace.RGB, False, 8, w, h, w * c)
                
                # Put the finished product in the output queue
                self.output_queue.append(pixbuf)
                
            except IndexError:
                # This is normal, means the input queue was empty
                time.sleep(0.005) # Sleep briefly to prevent busy-waiting
            except Exception as e:
                self.logger.error(f'Frame processing failed: {e}')

    def stop(self):
        """Signals the thread to stop."""
        self._is_running.clear()

# --- 2. The ROS Node: For Communication ---
class GuiLogicNode(Node):
    """Handles all ROS2 communication and pushes frames to the processing queue."""
    def __init__(self, frame_queue):
        super().__init__('gtk_gui_node')
        self.bridge = CvBridge()
        self.frame_queue = frame_queue # The queue for raw OpenCV frames

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.image_subscription = self.create_subscription(
            RosImage,
            '/hand_controller/image_annotated',
            self.image_callback,
            qos_profile)

    def image_callback(self, msg: RosImage):
        """Extremely lightweight callback. Just converts and queues the frame."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.frame_queue.append(cv_image)
        except Exception as e:
            self.get_logger().error(f'Image callback failed: {e}')

# --- 3. The GTK Window: For Display ---
class GtkWindow(Gtk.Window):
    """Handles all GUI elements and rendering."""
    def __init__(self):
        super().__init__(title="High-Performance ROS2 Camera Viewer")
        self.set_default_size(800, 600)
        self.connect("destroy", Gtk.main_quit)

        vbox = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=6, margin=6)
        self.add(vbox)

        self.image_view = Gtk.Image()
        vbox.pack_start(self.image_view, True, True, 0)

    def update_image(self, pixbuf):
        """Receives a finished GdkPixbuf and displays it."""
        self.image_view.set_from_pixbuf(pixbuf)

# --- 4. The Main Application Director ---
def main(args=None):
    rclpy.init(args=args)
    
    # --- Create the components ---
    window = GtkWindow()
    ros_node = GuiLogicNode(frame_queue=deque(maxlen=1))
    pixbuf_queue = deque(maxlen=1)
    
    # Create and start the dedicated worker thread
    worker = FrameProcessor(ros_node.frame_queue, pixbuf_queue, ros_node.get_logger())
    worker.start()

    # Set up the ROS executor
    executor = SingleThreadedExecutor()
    executor.add_node(ros_node)

    # --- Set up the event loops ---
    def gui_update_loop():
        """Pulls from the finished queue and updates the GUI. Runs in the GUI thread."""
        try:
            pixbuf = pixbuf_queue.popleft()
            window.update_image(pixbuf)
        except IndexError:
            pass # No new frame to display
        return True

    def ros_spin_loop():
        """Spins the ROS executor. Runs in the GUI thread."""
        executor.spin_once(timeout_sec=0)
        return True

    # Add loops to the GLib event manager
    GObject.timeout_add(16, gui_update_loop) # ~60fps for GUI updates
    GObject.timeout_add(10, ros_spin_loop)   # ~100Hz for ROS spinning

    window.show_all()

    try:
        Gtk.main()
    except KeyboardInterrupt:
        ros_node.get_logger().info("Keyboard interrupt received.")
    finally:
        ros_node.get_logger().info("Shutting down...")
        worker.stop()       # Signal the worker thread to exit
        worker.join(1.0)    # Wait for the worker to finish
        executor.shutdown()
        ros_node.destroy_node()

if __name__ == '__main__':
    main()
