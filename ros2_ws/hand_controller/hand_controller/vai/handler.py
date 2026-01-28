import pathlib
import subprocess
import os
from hand_controller.vai.qprofile import QProfProcess
import gi
import threading
from gi.repository import Gdk

from hand_controller.vai.common import (
    APP_NAME,
    CPU_THERMAL_KEY,
    CPU_UTIL_KEY,
    GPU_THERMAL_KEY,
    GPU_UTIL_KEY,
    MEM_THERMAL_KEY,
    MEM_UTIL_KEY,
    DSP_UTIL_KEY,
    HW_SAMPLING_PERIOD_ms,
    AUTOMATIC_DEMO_SWITCH_s,
    QUIT_CLEANUP_DELAY_ms
)
from hand_controller.vai.temp_profile import get_cpu_gpu_mem_temps

# Locks app version, prevents warnings
gi.require_version("Gtk", "3.0")
gi.require_version('Gst', '1.0') 

from gi.repository import GLib, Gtk, Gst

# needed to release gstreamer with cv2
os.environ['OPENCV_VIDEOIO_PRIORITY_MSMF'] = '0'

# Tuning variable to adjust the height of the video display
HEIGHT_OFFSET = 17
MAX_WINDOW_WIDTH = 1920 // 2
MAX_WINDOW_HEIGHT = 720
MIPI_CSI_CAMERA_SCAN_TIMEOUT = 5
CLOSE_APPLICATION_DELAY = 2

DUAL_WINDOW_DEMOS = ["add drop down items here if needed"]

class Handler:
    def __init__(self, display_fps_metrics=True):

        self.demoList = [
            None,
        ]

        self.QProf = QProfProcess()
        self.MainWindowShown = False
        self.MainWindow = None
        self.aboutWindow = None
        self.FPSRate0 = None
        self.FPSRate1 = None
        self.CPU_load = None
        self.GPU_load = None
        self.DSP_load = None
        self.MEM_load = None
        self.CPU_temp = None
        self.GPU_temp = None
        self.MEM_temp = None
        self.TopBox = None
        self.DataGrid = None
        self.BottomBox = None
        self.DrawArea1 = None
        self.DrawArea2 = None
        self.AspectFrame1 = None
        self.AspectFrame2 = None
        self.GraphDrawAreaTop = None
        self.GraphDrawAreaBottom = None
        self.demo_selection0 = None
        self.demo_selection1 = None
        self.display_fps_metrics = display_fps_metrics
        self.systemCameras = []
        self.dualDemoRunning0 = False
        self.dualDemoRunning1 = False
        self.CycleDemo0 = False
        self.CycleDemo1 = False
        self.demoSelection0Cnt = 0
        self.demoSelection1Cnt = 0

        # TODO: protect with sync primitive?
        self.sample_data = {
            CPU_UTIL_KEY: 0,
            MEM_UTIL_KEY: 0,
            GPU_UTIL_KEY: 0,
            DSP_UTIL_KEY: 0,
            CPU_THERMAL_KEY: 0,
            MEM_THERMAL_KEY: 0,
            GPU_THERMAL_KEY: 0,
        }
        GLib.timeout_add(HW_SAMPLING_PERIOD_ms, self.update_sample_data)

    def update_temps(self):
        if not self.sample_data:
            return GLib.SOURCE_REMOVE
        
        cpu_temp, gpu_temp, mem_temp = get_cpu_gpu_mem_temps()

        self.sample_data[CPU_THERMAL_KEY] = cpu_temp
        if cpu_temp is not None:
            GLib.idle_add(self.CPU_temp.set_text, "{:6.2f}".format(cpu_temp))
        self.sample_data[GPU_THERMAL_KEY] = gpu_temp
        if gpu_temp is not None:
            GLib.idle_add(self.GPU_temp.set_text, "{:6.2f}".format(gpu_temp))
        self.sample_data[MEM_THERMAL_KEY] = mem_temp
        if mem_temp is not None:
            GLib.idle_add(self.MEM_temp.set_text, "{:6.2f}".format(mem_temp))

        return GLib.SOURCE_REMOVE

    def update_loads(self):
        if not self.sample_data:
            return GLib.SOURCE_REMOVE

        cpu_util, gpu_util, mem_util, dsp_util = (
            self.QProf.get_cpu_usage_pct(),
            self.QProf.get_gpu_usage_pct(),
            self.QProf.get_memory_usage_pct(),
            self.QProf.get_dsp_usage_pct(),
        )
        self.sample_data[CPU_UTIL_KEY] = cpu_util
        self.sample_data[GPU_UTIL_KEY] = gpu_util
        self.sample_data[MEM_UTIL_KEY] = mem_util
        self.sample_data[DSP_UTIL_KEY] = dsp_util
        GLib.idle_add(self.CPU_load.set_text, "{:6.2f}".format(cpu_util))
        GLib.idle_add(self.GPU_load.set_text, "{:6.2f}".format(gpu_util))
        GLib.idle_add(self.MEM_load.set_text, "{:6.2f}".format(mem_util))
        GLib.idle_add(self.DSP_load.set_text, "{:6.2f}".format(dsp_util))
        return GLib.SOURCE_REMOVE

    def update_sample_data(self):
        # Run blocking I/O in separate threads to avoid freezing the UI.
        # The update functions will then schedule UI updates on the main thread.
        threading.Thread(target=self.update_temps, daemon=True).start()
        threading.Thread(target=self.update_loads, daemon=True).start()
        return GLib.SOURCE_CONTINUE

    def close_about(self, *args):
        if self.aboutWindow:
            self.aboutWindow.hide()

    def open_about(self, *args):
        if self.aboutWindow:
            self.aboutWindow.set_transient_for(self.MainWindow)
            self.aboutWindow.run()

    def on_mainWindow_destroy(self, *args):
        """Handle exit signals and clean up resources before exiting the application.

        Due to the threaded nature of the application, this function needs to be carefully linked with Gtk
        """
        print("Shutdown initiated...")
        if self.QProf is not None:
            self.QProf.Close()

        # Schedule the final shutdown sequence.
        GLib.timeout_add(QUIT_CLEANUP_DELAY_ms, self.quit_application, *args)

    def quit_application(self, *args):
        print("Waiting for background threads to join...")
        # Join threads to ensure they exit cleanly before the main process
        if self.QProf and self.QProf.is_alive():
            self.QProf.Close()
            self.QProf.join(timeout=2.0)

        print("Exiting GTK main loop.")
        Gtk.main_quit(*args)
        return GLib.SOURCE_REMOVE # Stop the timer

    def close_dialog(self, *args):
        if self.dialogWindow:
            self.dialogWindow.hide()
        return GLib.SOURCE_REMOVE

    def show_message(self):
        if self.dialogWindow:
            self.dialogWindow.set_transient_for(self.MainWindow)
            self.dialogWindow.show_all()
        return GLib.SOURCE_REMOVE

    def on_mainWindow_show(self, *args):
        if not self.MainWindowShown:
            self.MainWindowShown = True
            #GLib.idle_add(self.show_message)
