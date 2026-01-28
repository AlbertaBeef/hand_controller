# gtk_gui_node.py

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from sensor_msgs.msg import Image as RosImage  # Import ROS Image message
from cv_bridge import CvBridge               # To convert between ROS and OpenCV images
import cv2                                   # OpenCV library
import numpy as np                           # NumPy for array manipulation

import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, GObject, GLib, GdkPixbuf

# --- ROS2 Node Class ---
# Handles all ROS2 logic, now including image subscription.
class GuiLogicNode(Node):
    def __init__(self, gui_window):
        super().__init__('gtk_gui_node')
        self.gui = gui_window
        self.bridge = CvBridge()  # Initialize CvBridge

        # --- ROS2 Publishers and Subscribers ---
        self.publisher_ = self.create_publisher(String, 'gui_button_clicks', 10)
        
        self.status_subscription = self.create_subscription(
            String,
            'gui_status_topic',
            self.status_callback,
            10)
            
        # New subscriber for the annotated image stream
        self.image_subscription = self.create_subscription(
            RosImage,
            '/hand_controller/image_annotated',  # The topic to view
            self.image_callback,
            10) # QoS profile depth

    def status_callback(self, msg: String):
        GLib.idle_add(self.gui.update_status_label, f"Status: {msg.data}")

    def image_callback(self, msg: RosImage):
        """Callback for receiving a ROS Image message."""
        try:
            # Convert the ROS Image message to an OpenCV image (BGR format)
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Convert the OpenCV image to a GdkPixbuf for GTK display.
            # OpenCV uses BGR, but GdkPixbuf expects RGB.
            h, w, c = cv_image.shape
            # Create a Pixbuf from the NumPy array data
            pixbuf = GdkPixbuf.Pixbuf.new_from_data(
                cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB).tobytes(),
                GdkPixbuf.Colorspace.RGB,
                False,  # has_alpha
                8,      # bits_per_sample
                w, h,   # width, height
                w * c   # rowstride
            )
            
            # Call the GUI update method in a thread-safe way
            GLib.idle_add(self.gui.update_image_view, pixbuf)
            
        except Exception as e:
            self.get_logger().error(f'Failed to process image: {e}')

    def on_button_clicked(self):
        msg = String()
        msg.data = f"Button clicked at {self.get_clock().now().to_msg().sec}"
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

# --- GTK Window Class ---
# Handles all GUI logic, now including an image widget.
class GtkWindow(Gtk.Window):
    def __init__(self):
        super().__init__(title="GTK+ 3 ROS2 Node with Camera")
        self.set_default_size(800, 600)  # Increased window size for the camera view
        self.ros_node = None

        # Use a vertical box to stack the camera view and controls
        vbox = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=10)
        vbox.set_margin_top(10)
        vbox.set_margin_bottom(10)
        vbox.set_margin_start(10)
        vbox.set_margin_end(10)
        self.add(vbox)

        # --- New Gtk.Image widget for the camera stream ---
        self.image_view = Gtk.Image()
        self.image_view.set_hexpand(True)
        self.image_view.set_vexpand(True)
        vbox.pack_start(self.image_view, True, True, 0)

        # --- Control widgets at the bottom ---
        controls_grid = Gtk.Grid(column_spacing=10, row_spacing=10)
        vbox.pack_start(controls_grid, False, False, 0)

        self.status_label = Gtk.Label(label="Waiting for status...")
        self.status_label.set_hexpand(True)

        self.click_button = Gtk.Button(label="Publish Message")
        self.click_button.connect("clicked", self.button_callback)
        
        controls_grid.attach(self.status_label, 0, 0, 1, 1)
        controls_grid.attach(self.click_button, 1, 0, 1, 1)

    def button_callback(self, widget):
        if self.ros_node:
            self.ros_node.on_button_clicked()

    def update_status_label(self, text):
        self.status_label.set_text(text)
        
    def update_image_view(self, pixbuf):
        """Thread-safe method to update the Gtk.Image widget."""
        self.image_view.set_from_pixbuf(pixbuf)

# --- Main Function ---
# (This function remains unchanged)
def main(args=None):
    rclpy.init(args=args)
    window = GtkWindow()
    ros_node = GuiLogicNode(gui_window=window)
    window.ros_node = ros_node
    window.connect("destroy", Gtk.main_quit)
    window.show_all()
    executor = SingleThreadedExecutor()
    executor.add_node(ros_node)

    def ros_spin():
        executor.spin_once(timeout_sec=0.01)
        return True

    GObject.timeout_add(100, ros_spin)
    try:
        Gtk.main()
    except KeyboardInterrupt:
        ros_node.get_logger().info("Keyboard interrupt, shutting down.")
    finally:
        ros_node.get_logger().info("Shutting down.")
        executor.shutdown()
        ros_node.destroy_node()

if __name__ == '__main__':
    main()
