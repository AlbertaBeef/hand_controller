#!/usr/bin/env python3
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from sensor_msgs.msg import Image as RosImage  # Import ROS Image message
from geometry_msgs.msg import Twist            # For velocity commands
from cv_bridge import CvBridge               # To convert between ROS and OpenCV images
import cv2                                   # OpenCV library
import numpy as np                           # NumPy for array manipulation

import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, GObject, GLib, GdkPixbuf


import collections
import os
import threading
import time
import gi
#import pyopencl as cl


from hand_controller.vai.common import (APP_HEADER, CPU_THERMAL_KEY, CPU_UTIL_KEY,
                        GPU_THERMAL_KEY, GPU_UTIL_KEY, GRAPH_SAMPLE_SIZE,
                        MEM_THERMAL_KEY, MEM_UTIL_KEY, DSP_UTIL_KEY, TIME_KEY, 
                        TRIA, TRIA_BLUE_RGBH, TRIA_PINK_RGBH, TRIA_YELLOW_RGBH, 
                        TRIA_GREEN_RGBH, GRAPH_SAMPLE_WINDOW_SIZE_s,
                        get_ema)
from hand_controller.vai.graphing import (draw_axes_and_labels,
                          draw_graph_background_and_border, draw_graph_data)
from hand_controller.vai.handler import Handler
from hand_controller.vai.qprofile import QProfProcess

package_name = 'hand_controller'
package_share_directory = get_package_share_directory(package_name)
latest_hand_data = None

class FdFilter:
    """
    Redirects low-level file descriptors (stdout, stderr) to a pipe,
    filters the output in a separate thread, and writes the filtered
    output back to the original destination. This is necessary to
    suppress messages from C libraries that write directly to file
    descriptors, bypassing sys.stdout/sys.stderr.
    """
    def __init__(self, filter_strings):
        self.filter_strings = [s.lower() for s in filter_strings]
        self.original_stdout_fd = os.dup(1)
        self.original_stderr_fd = os.dup(2)

        # Create pipes to intercept stdout and stderr
        self.stdout_pipe_r, self.stdout_pipe_w = os.pipe()
        self.stderr_pipe_r, self.stderr_pipe_w = os.pipe()

        # Redirect stdout and stderr to the write-ends of the pipes
        os.dup2(self.stdout_pipe_w, 1)
        os.dup2(self.stderr_pipe_w, 2)

        # Create threads to read from the pipes, filter, and write to original FDs
        self.stdout_thread = threading.Thread(target=self._pipe_reader, args=(self.stdout_pipe_r, self.original_stdout_fd))
        self.stderr_thread = threading.Thread(target=self._pipe_reader, args=(self.stderr_pipe_r, self.original_stderr_fd))
        self.stdout_thread.daemon = True
        self.stderr_thread.daemon = True
        self.stdout_thread.start()
        self.stderr_thread.start()

    def _pipe_reader(self, pipe_r_fd, original_dest_fd):
        """Reads from a pipe, filters, and writes to the destination."""
        with os.fdopen(pipe_r_fd, 'r') as pipe_file:
            for line in iter(pipe_file.readline, ''):
                if not any(f in line.lower() for f in self.filter_strings):
                    os.write(original_dest_fd, line.encode('utf-8'))

# Locks app version, prevents warnings
gi.require_version("Gdk", "3.0")
gi.require_version("Gst", "1.0")
gi.require_version("Gtk", "3.0")
from gi.repository import Gdk, Gst, Gtk

# --- Graphing constants ---

UTIL_GRAPH_COLORS_RGBF = {
    CPU_UTIL_KEY: tuple(c / 255.0 for c in TRIA_PINK_RGBH),
    MEM_UTIL_KEY: tuple(c / 255.0 for c in TRIA_BLUE_RGBH),
    GPU_UTIL_KEY: tuple(c / 255.0 for c in TRIA_YELLOW_RGBH),
    DSP_UTIL_KEY: tuple(c / 255.0 for c in TRIA_GREEN_RGBH),
}

THERMAL_GRAPH_COLORS_RGBF = {
    CPU_THERMAL_KEY: tuple(c / 255.0 for c in TRIA_PINK_RGBH),
    MEM_THERMAL_KEY: tuple(c / 255.0 for c in TRIA_BLUE_RGBH),
    GPU_THERMAL_KEY: tuple(c / 255.0 for c in TRIA_YELLOW_RGBH),
}

GRAPH_LABEL_FONT_SIZE = 14
MAX_TIME_DISPLAYED = 0
MIN_TEMP_DISPLAYED = 35
MAX_TEMP_DISPLAYED = 95
MIN_UTIL_DISPLAYED = 0
MAX_UTIL_DISPLAYED = 100

# --- End Graphing constants ---
def is_monitor_above_2k():
    """
    Checks if any connected monitor has a native resolution greater than 2K (2560x1440).
    Uses EDID data from /sys/class/drm/ to determine resolution.
    
    Returns:
        bool: True if any monitor has resolution > 2560x1440, False otherwise.
    """
    drm_path = '/sys/class/drm/'
    above_2k = False

    try:
        for device in os.listdir(drm_path):
            # Look for connected display devices (e.g., card0-HDMI-A-1, card0-eDP-1)
            if not device.startswith('card'):
                continue

            status_file = os.path.join(drm_path, device, 'status')
            edid_file = os.path.join(drm_path, device, 'edid')

            # Only check if the monitor is connected
            if os.path.exists(status_file) and os.path.exists(edid_file):
                with open(status_file, 'r') as f:
                    if f.read().strip() != 'connected':
                        continue

                # Read EDID data
                with open(edid_file, 'rb') as f:
                    edid_data = f.read()

                if len(edid_data) < 128:
                    continue  # Invalid EDID

                # Parse EDID to get the native resolution
                # Detailed Timing Descriptor 1 starts at 54th byte
                dtd_start = 54
                if dtd_start + 18 <= len(edid_data):
                    # First DTD (usually the preferred/native mode)
                    dtd = edid_data[dtd_start:dtd_start+18]

                    # Parse horizontal active pixels (bytes 2-3)
                    h_active_lo = dtd[2]
                    h_active_hi = (dtd[4] & 0xF0) >> 4
                    width = h_active_lo + (h_active_hi << 8)

                    # Parse vertical active lines (bytes 5-6)
                    v_active_lo = dtd[5]
                    v_active_hi = (dtd[7] & 0xF0) >> 4
                    height = v_active_lo + (v_active_hi << 8)

                    # Check if resolution is greater than 2K (2560x1440)
                    if width > 2560 or height > 1440:
                        # Confirm it's a valid resolution
                        if width >= 3840 or height >= 2160:
                            above_2k = True
                            break  # Found a 4K or higher display

    except Exception as e:
        print(f"Error reading EDID: {e}")
        return False

    return above_2k

GladeBuilder = Gtk.Builder()

if is_monitor_above_2k():
    print("Connected monitor resolution is above 2K (e.g., 4K).")
    RESOURCE_FOLDER = os.path.join(package_share_directory, "resources_high")
else:
    print("No monitor above 2K resolution detected.")
    RESOURCE_FOLDER = os.path.join(package_share_directory, "resources_low")

LAYOUT_PATH = os.path.join(RESOURCE_FOLDER, "GSTLauncher.glade")

def get_min_time_delta_smoothed(time_series: list):
    """Returns the delta from the current time to the first entry in the time series. If the time series is empty, returns 0."""
    if not time_series: return 0

    x_min = -int(time.monotonic() - time_series[0])

    # Help with the jittering of the graph
    if abs(x_min - GRAPH_SAMPLE_WINDOW_SIZE_s) <= 1:
        x_min = -GRAPH_SAMPLE_WINDOW_SIZE_s

    return x_min


# --- ROS2 Node Class ---
# Handles all ROS2 logic, now including image subscription.
class GuiLogicNode(Node):
    def __init__(self, gui_window):
        super().__init__('gtk_gui_node')
        self.gui = gui_window
        self.message_count = 0
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
        
        # Subscription for Twist data (velocity commands)
        self.twist_subscription = self.create_subscription(
            Twist,
            '/hand_controller/cmd_vel',
            self.twist_callback,
            10)

        # Subscription for Twist data (velocity commands)
        self.twist_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.twist_callback,
            10)

    def status_callback(self, msg: String):
        GLib.idle_add(self.gui.update_status_label, f"Status: {msg.data}")
    '''
    def image_callback(self, msg: RosImage):
        try:
            # Convert ROS Image to OpenCV (BGR)
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            h, w = cv_image.shape[:2]

            # Upload to GPU (as BGRA for alignment)
            cv_image_rgba = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGBA)
            img_gpu = cl.image_from_array(self.ctx, cv_image_rgba, 4)

            # Output buffer
            dest_rgba = np.empty_like(cv_image_rgba)
            dest_gpu = cl.Buffer(self.ctx, cl.mem_flags.WRITE_ONLY, dest_rgba.nbytes)

            # Run GPU kernel
            self.prg.bgr_to_rgb(self.queue, (w, h), None, dest_gpu, np.int32(w), np.int32(h))

            # Read back result
            cl.enqueue_copy(self.queue, dest_rgba, dest_gpu)
            rgb_image = cv2.cvtColor(dest_rgba, cv2.COLOR_RGBA2RGB)

            # Convert to Pixbuf
            pixbuf = GdkPixbuf.Pixbuf.new_from_data(
                rgb_image.tobytes(),
                GdkPixbuf.Colorspace.RGB, False, 8, w, h, w * 3
            )
            GLib.idle_add(self.gui.update_image_view, pixbuf)

        except Exception as e:
            print(f"Error in image callback: {e}")

    '''
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

    def twist_callback(self, twist_msg: Twist):
        """Callback for receiving Twist messages."""
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
            latest_hand_data = hand_data
                
        except Exception as e:
            self.get_logger().error(f"Error processing hand data: {e}")



class VaiDemoManager:
    def __init__(self):
        self.eventHandler = Handler()
        self.running = True
        self.ros_node = None
        self.set_video_sink0 = None
        self.leftHandStatus = None
        self.rightHandStatus = None
        self.leftHandImage = None
        self.rightHandImage = None
        self.leftHandImageLoaded = None
        self.rightHandImageLoaded = None
        self.leftHandActive = False
        self.rightHandActive = False

        GLib.idle_add(self.update_hand_data)

    def update_image_view(self, pixbuf):
        """Thread-safe method to update the Gtk.Image widget."""
        if pixbuf:
            width = self.set_video_sink0.get_allocated_width()
            height = self.set_video_sink0.get_allocated_height()

            # Only scale if the widget has a size and the pixbuf size is different
            if width > 0 and height > 0 and (pixbuf.get_width() != width or pixbuf.get_height() != height):
                # GdkPixbuf.InterpType.BILINEAR is a good compromise between speed and quality.
                # For higher quality, you could use GdkPixbuf.InterpType.HYPER.
                scaled_pixbuf = pixbuf.scale_simple(width, height, GdkPixbuf.InterpType.BILINEAR)
                self.set_video_sink0.set_from_pixbuf(scaled_pixbuf)
            else:
                self.set_video_sink0.set_from_pixbuf(pixbuf)

    def update_hand_data(self):
        global latest_hand_data
        if latest_hand_data == None:
            return GLib.SOURCE_CONTINUE
                                
        left_hand_active   = latest_hand_data['left_hand']['active']
        right_hand_active  = latest_hand_data['right_hand']['active']
        normalized_angular = latest_hand_data['left_hand']['x_position']
        normalized_linear  = latest_hand_data['right_hand']['z_position']

        if self.leftHandActive != left_hand_active:
            self.leftHandActive = left_hand_active
            
            if self.leftHandActive:
                left_status_text = "Active"  
                GLib.idle_add(self.leftHandStatus.get_style_context().remove_class, "status")   
                GLib.idle_add(self.leftHandStatus.get_style_context().add_class, "status_active")
            else:
                left_status_text = "Inactive"
                GLib.idle_add(self.leftHandStatus.get_style_context().remove_class, "status_active")   
                GLib.idle_add(self.leftHandStatus.get_style_context().add_class, "status")


            GLib.idle_add(self.leftHandStatus.set_text, f"Status: {left_status_text}")

        if self.rightHandActive != right_hand_active:
            self.rightHandActive = right_hand_active

            if self.rightHandActive:
                right_status_text = "Active"
                GLib.idle_add(self.rightHandStatus.get_style_context().remove_class, "status")   
                GLib.idle_add(self.rightHandStatus.get_style_context().add_class, "status_active")
            else:
                right_status_text = "Inactive"
                GLib.idle_add(self.rightHandStatus.get_style_context().remove_class, "status_active")   
                GLib.idle_add(self.rightHandStatus.get_style_context().add_class, "status")

            GLib.idle_add(self.rightHandStatus.set_text, f"Status: {right_status_text}")
        
        if not left_hand_active:
            loadLeftImage = "leftright.png"
        elif (normalized_angular > 0.1):
            loadLeftImage = "left.png"
        elif (normalized_angular < -0.1):
            loadLeftImage = "right.png"
        else:
            loadLeftImage = "leftright.png"

        if loadLeftImage != self.leftHandImageLoaded:
            self.leftHandImageLoaded = loadLeftImage
            GLib.idle_add(self.leftHandImage.set_from_file, os.path.join(RESOURCE_FOLDER, self.leftHandImageLoaded))


        if not right_hand_active:
            loadrightImage = "updown.png"
        elif (normalized_linear > 0.1):
            loadrightImage = "up.png"
        elif (normalized_linear < -0.1):
            loadrightImage = "down.png"
        else:
            loadrightImage = "updown.png"

        if loadrightImage != self.rightHandImageLoaded:
            self.rightHandImageLoaded = loadrightImage
            GLib.idle_add(self.rightHandImage.set_from_file, os.path.join(RESOURCE_FOLDER, self.rightHandImageLoaded))

        return GLib.SOURCE_CONTINUE                

    def resize_graphs_dynamically(self, parent_widget, _allocation):
        if not self.eventHandler.GraphDrawAreaTop or not self.eventHandler.GraphDrawAreaBottom:
            return
        
        """Resize graphing areas to be uniform and fill remaining space. To be called on size-allocate signal."""

        # Total width will be a function of the current lifecycle of the widget, it may have a surprising value
        total_width = parent_widget.get_allocated_width()
        total_height = parent_widget.get_allocated_height()

        self.main_window_dims = (total_width, total_height)
        if total_width == 0:
            return

        BottomBox = GladeBuilder.get_object("BottomBox")
        if not BottomBox:
            return

        BottomBox_width = BottomBox.get_allocated_width()
        if BottomBox_width == 0:
            return        

        # These datagrid widths are what determine the remaining space
        data_grid = GladeBuilder.get_object("DataGrid")
        data_grid1 = GladeBuilder.get_object("DataGrid1")
        if not data_grid or not data_grid1:
            return

        remaining_graph_width = BottomBox_width - (
            data_grid.get_allocated_width() + data_grid1.get_allocated_width()
        )
        # Account for margins that arent included in the allocated width
        remaining_graph_width -= (
            data_grid.get_margin_start() + data_grid.get_margin_end() + 10
        )
        remaining_graph_width -= (
            data_grid1.get_margin_start() + data_grid1.get_margin_end() + 10
        )

        half = remaining_graph_width // 2
        if half < 0:
            return

        try:
            window_x, window_y = self.eventHandler.DrawArea1.translate_coordinates(self.eventHandler.DrawArea1.get_toplevel(), 0, 0)

            camera_bottom_position = window_y + self.eventHandler.DrawArea1.get_allocated_height()

            if camera_bottom_position > 148:
                BottomBox.set_size_request(-1, round(total_height - camera_bottom_position))
        except:
            pass

        graph_top = self.eventHandler.GraphDrawAreaTop
        graph_bottom = self.eventHandler.GraphDrawAreaBottom
        # Only resize if changed, otherwise it can cause a loop
        if (
            graph_top.get_allocated_width() != half
            or graph_bottom.get_allocated_width() != half
        ):
            graph_top.set_size_request(half, -1)
            graph_bottom.set_size_request(half, -1)

    def init_graph_data(self, sample_size=GRAPH_SAMPLE_SIZE):
        """Initialize the graph data according to graph box size"""
        self.util_data = {
            TIME_KEY: collections.deque([], maxlen=sample_size),
            CPU_UTIL_KEY: collections.deque([], maxlen=sample_size),
            MEM_UTIL_KEY: collections.deque([], maxlen=sample_size),
            GPU_UTIL_KEY: collections.deque([], maxlen=sample_size),
            DSP_UTIL_KEY: collections.deque([], maxlen=sample_size),
        }
        self.thermal_data = {
            TIME_KEY: collections.deque([], maxlen=sample_size),
            CPU_THERMAL_KEY: collections.deque([], maxlen=sample_size),
            MEM_THERMAL_KEY: collections.deque([], maxlen=sample_size),
            GPU_THERMAL_KEY: collections.deque([], maxlen=sample_size),
        }

    def _sample_util_data(self):
        """Sample the utilization data; prefer this function because it timestamps entries to util data"""

        if self.util_data is None or self.thermal_data is None:
            self.init_graph_data()

        self.util_data[TIME_KEY].append(time.monotonic())

        # Sample and smooth the data with exponential smoothing
        cur_cpu = self.eventHandler.sample_data[CPU_UTIL_KEY]
        cur_gpu = self.eventHandler.sample_data[GPU_UTIL_KEY]
        cur_mem = self.eventHandler.sample_data[MEM_UTIL_KEY]
        cur_dsp = self.eventHandler.sample_data[DSP_UTIL_KEY]

        last_cpu = self.util_data[CPU_UTIL_KEY][-1] if self.util_data[CPU_UTIL_KEY] else cur_cpu
        last_gpu = self.util_data[GPU_UTIL_KEY][-1] if self.util_data[GPU_UTIL_KEY] else cur_gpu
        last_mem = self.util_data[MEM_UTIL_KEY][-1] if self.util_data[MEM_UTIL_KEY] else cur_mem
        last_dsp = self.util_data[DSP_UTIL_KEY][-1] if self.util_data[DSP_UTIL_KEY] else cur_dsp

        ema_cpu = get_ema(cur_cpu, last_cpu)
        ema_gpu = get_ema(cur_gpu, last_gpu)
        ema_mem = get_ema(cur_mem, last_mem)
        ema_dsp = get_ema(cur_dsp, last_dsp)

        self.util_data[CPU_UTIL_KEY].append(ema_cpu)
        self.util_data[GPU_UTIL_KEY].append(ema_gpu)
        self.util_data[MEM_UTIL_KEY].append(ema_mem)
        self.util_data[DSP_UTIL_KEY].append(ema_dsp)

        cur_time = time.monotonic()
        while (
            self.util_data[TIME_KEY]
            and cur_time - self.util_data[TIME_KEY][0] > GRAPH_SAMPLE_WINDOW_SIZE_s
        ):
            self.util_data[TIME_KEY].popleft()
            self.util_data[CPU_UTIL_KEY].popleft()
            self.util_data[GPU_UTIL_KEY].popleft()
            self.util_data[MEM_UTIL_KEY].popleft()
            self.util_data[DSP_UTIL_KEY].popleft()

    def on_util_graph_draw(self, widget, cr):
        if not self.eventHandler.GraphDrawAreaTop:
            return True

        if not self.util_data:
            self.eventHandler.GraphDrawAreaTop.queue_draw()
            return True
        
        """Draw the util graph on the draw area"""

        self._sample_util_data()

        width = widget.get_allocated_width()
        height = widget.get_allocated_height()

        draw_graph_background_and_border(
            width, height, cr, res_tuple=self.main_window_dims
        )

        x_min = get_min_time_delta_smoothed(self.util_data[TIME_KEY])

        x_lim = (x_min, MAX_TIME_DISPLAYED)
        y_lim = (MIN_UTIL_DISPLAYED, MAX_UTIL_DISPLAYED)

        x_axis, y_axis = draw_axes_and_labels(
            cr,
            width,
            height,
            x_lim,
            y_lim,
            x_ticks=4,
            y_ticks=2,
            dynamic_margin=True,
            x_label="seconds",
            y_label="%",
            res_tuple=self.main_window_dims,
        )
        draw_graph_data(
            self.util_data,
            UTIL_GRAPH_COLORS_RGBF,
            x_axis,
            y_axis,
            cr,
            y_lim=y_lim,
            res_tuple=self.main_window_dims,
        )

        self.eventHandler.GraphDrawAreaTop.queue_draw()

        return True

    def _sample_thermal_data(self):
        """Sample the thermal data; prefer this function because it timestamps entries to thermal data"""
        if self.thermal_data is None:
            self.init_graph_data()

        self.thermal_data[TIME_KEY].append(time.monotonic())

        # Sample and smooth the data with exponential smoothing
        cur_cpu = self.eventHandler.sample_data[CPU_THERMAL_KEY]
        cur_gpu = self.eventHandler.sample_data[GPU_THERMAL_KEY]
        cur_mem = self.eventHandler.sample_data[MEM_THERMAL_KEY]

        last_cpu = self.thermal_data[CPU_THERMAL_KEY][-1] if self.thermal_data[CPU_THERMAL_KEY] else cur_cpu
        last_gpu = self.thermal_data[GPU_THERMAL_KEY][-1] if self.thermal_data[GPU_THERMAL_KEY] else cur_gpu
        last_mem = self.thermal_data[MEM_THERMAL_KEY][-1] if self.thermal_data[MEM_THERMAL_KEY] else cur_mem

        ema_cpu = get_ema(cur_cpu, last_cpu)
        ema_gpu = get_ema(cur_gpu, last_gpu)
        ema_mem = get_ema(cur_mem, last_mem)

        self.thermal_data[CPU_THERMAL_KEY].append(
            ema_cpu
        )
        self.thermal_data[GPU_THERMAL_KEY].append(
            ema_gpu
        )
        self.thermal_data[MEM_THERMAL_KEY].append(
            ema_mem
        )

        cur_time = time.monotonic()
        while (
            self.thermal_data[TIME_KEY]
            and cur_time - self.thermal_data[TIME_KEY][0] > GRAPH_SAMPLE_WINDOW_SIZE_s
        ):
            self.thermal_data[TIME_KEY].popleft()
            self.thermal_data[CPU_THERMAL_KEY].popleft()
            self.thermal_data[GPU_THERMAL_KEY].popleft()
            self.thermal_data[MEM_THERMAL_KEY].popleft()

    def on_thermal_graph_draw(self, widget, cr):
        if not self.eventHandler.GraphDrawAreaBottom:
            return
        
        if not self.thermal_data:
            self.eventHandler.GraphDrawAreaBottom.queue_draw()
            return True    
            
        """Draw the graph on the draw area"""

        self._sample_thermal_data()

        width = widget.get_allocated_width()
        height = widget.get_allocated_height()

        draw_graph_background_and_border(
            width, height, cr, res_tuple=self.main_window_dims
        )
        x_min = get_min_time_delta_smoothed(self.thermal_data[TIME_KEY])
        x_lim = (x_min, MAX_TIME_DISPLAYED)
        y_lim = (MIN_TEMP_DISPLAYED, MAX_TEMP_DISPLAYED)

        x_axis, y_axis = draw_axes_and_labels(
            cr,
            width,
            height,
            x_lim,
            y_lim,
            x_ticks=4,
            y_ticks=2,
            dynamic_margin=True,
            x_label="seconds",
            y_label="°C",
            res_tuple=self.main_window_dims,
        )
        draw_graph_data(
            self.thermal_data,
            THERMAL_GRAPH_COLORS_RGBF,
            x_axis,
            y_axis,
            cr,
            y_lim=y_lim,
            res_tuple=self.main_window_dims,
        )

        self.eventHandler.GraphDrawAreaBottom.queue_draw()
        return True

    def localApp(self):
        global GladeBuilder

        # Initialize GStreamer. The log level is now controlled by the GST_DEBUG environment variable.
        Gst.init(None)

        self.init_graph_data()

        """Build application window and connect signals"""
        GladeBuilder.add_from_file(LAYOUT_PATH)
        GladeBuilder.connect_signals(self.eventHandler)

        screen = Gdk.Screen.get_default()
        provider = Gtk.CssProvider()
        provider.load_from_path(os.path.join(RESOURCE_FOLDER, "app.css"))
        Gtk.StyleContext.add_provider_for_screen(
            screen, provider, Gtk.STYLE_PROVIDER_PRIORITY_APPLICATION
        )

        self.eventHandler.MainWindow = GladeBuilder.get_object("mainWindow")
        self.eventHandler.MainWindow.connect(
            "size-allocate", self.resize_graphs_dynamically
        )
        self.eventHandler.aboutWindow = GladeBuilder.get_object("aboutWindow")
        self.eventHandler.FPSRate0 = GladeBuilder.get_object("FPS_rate_0")
        self.eventHandler.FPSRate1 = GladeBuilder.get_object("FPS_rate_1")
        self.eventHandler.CPU_load = GladeBuilder.get_object("CPU_load")
        self.eventHandler.GPU_load = GladeBuilder.get_object("GPU_load")
        self.eventHandler.DSP_load = GladeBuilder.get_object("DSP_load")
        self.eventHandler.MEM_load = GladeBuilder.get_object("MEM_load")
        self.eventHandler.CPU_temp = GladeBuilder.get_object("CPU_temp")
        self.eventHandler.GPU_temp = GladeBuilder.get_object("GPU_temp")
        self.eventHandler.MEM_temp = GladeBuilder.get_object("MEM_temp")
        self.eventHandler.TopBox = GladeBuilder.get_object("TopBox")
        self.eventHandler.DataGrid = GladeBuilder.get_object("DataGrid")
        self.eventHandler.BottomBox = GladeBuilder.get_object("BottomBox")
        self.eventHandler.DrawArea1 = GladeBuilder.get_object("videosink0")
        self.eventHandler.DrawArea2 = GladeBuilder.get_object("videosink1")
        
        self.set_video_sink0 = GladeBuilder.get_object("videosink0")
        self.leftHandStatus  = GladeBuilder.get_object("leftHandStatus")
        self.rightHandStatus = GladeBuilder.get_object("rightHandStatus")
        self.leftHandImage = GladeBuilder.get_object("leftHandImage")
        self.rightHandImage = GladeBuilder.get_object("rightHandImage")

        self.eventHandler.GraphDrawAreaTop = GladeBuilder.get_object("GraphDrawAreaTop")
        self.eventHandler.GraphDrawAreaBottom = GladeBuilder.get_object("GraphDrawAreaBottom")

        self.eventHandler.dialogWindow = GladeBuilder.get_object("dialogWindow")

        # TODO: Dynamic sizing, positioning
        self.eventHandler.GraphDrawAreaTop.connect("draw", self.on_util_graph_draw)
        self.eventHandler.GraphDrawAreaBottom.connect("draw", self.on_thermal_graph_draw)

        self.eventHandler.QProf = QProfProcess()
        self.eventHandler.QProf.daemon = True # Ensure thread doesn't block app exit

        # TODO: Can just put these in CSS
        #self.eventHandler.MainWindow.override_background_color(
        #    Gtk.StateFlags.NORMAL, Gdk.RGBA(23 / 255, 23 / 255, 23 / 255, 0)
        #)
        #self.eventHandler.TopBox.override_background_color(
        #    Gtk.StateType.NORMAL, Gdk.RGBA(23 / 255, 23 / 255, 23 / 255, 0.5)
        #)
        #self.eventHandler.BottomBox.override_background_color(
        #    Gtk.StateType.NORMAL, Gdk.RGBA(0 / 255, 23 / 255, 23 / 255, 1)
        #)

        self.eventHandler.MainWindow.set_decorated(False)
        self.eventHandler.MainWindow.set_keep_below(True)
        self.eventHandler.MainWindow.maximize()
        self.eventHandler.MainWindow.show_all()


        self.eventHandler.QProf.start()

        settings = Gtk.Settings.get_default()
        settings.set_property("gtk-cursor-theme-name","Adwaita")
        settings.set_property("gtk-cursor-theme-size", 32)

        # --- Filter unwanted log messages from ML plugin ---
        # The QNN plugin prints log messages that can't be suppressed via
        # environment variables, so we filter them from the low-level
        # file descriptors directly.
        filter_list = [
            "<W> No usable logger handle was found",
            "<W> Logs will be sent to the system's default channel",
            "Could not find ncvt for conv cost",
            "Could not find conv_ctrl for conv cost"
        ]
        self.log_filter = FdFilter(filter_list)
        # --- End Filter ---

# --- Main Function ---
# (This function remains unchanged)
def main(args=None):
    rclpy.init(args=args)
    print(TRIA)
    print(f"\nLaunching {APP_HEADER}")
    # Create the video object
    # Add port= if is necessary to use a different one
    window = VaiDemoManager()
    window.localApp()
    ros_node = GuiLogicNode(gui_window=window)
    window.ros_node = ros_node
    executor = SingleThreadedExecutor()
    executor.add_node(ros_node)

    def ros_spin():
        executor.spin_once(timeout_sec=0.01)
        return True

    GObject.timeout_add(30, ros_spin)
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
