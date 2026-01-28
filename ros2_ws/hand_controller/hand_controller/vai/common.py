"""Common utilities and constants for VAI demo"""

import subprocess

GRAPH_SAMPLE_WINDOW_SIZE_s = 31
HW_SAMPLING_PERIOD_ms = 250
GRAPH_DRAW_PERIOD_ms = 30
AUTOMATIC_DEMO_SWITCH_s = 60
QUIT_CLEANUP_DELAY_ms = 1000

GRAPH_SAMPLE_SIZE = int(GRAPH_SAMPLE_WINDOW_SIZE_s * 1000 / GRAPH_DRAW_PERIOD_ms)

TIME_KEY = "time"
CPU_UTIL_KEY = "cpu %"
MEM_UTIL_KEY = "lpddr5 %"
GPU_UTIL_KEY = "gpu %"
DSP_UTIL_KEY = "dsp %"
CPU_THERMAL_KEY = "cpu temp (°c)"
MEM_THERMAL_KEY = "lpddr5 temp (°c)"
GPU_THERMAL_KEY = "gpu temp (°c)"

# Triadic colors, indexed on Tria pink
TRIA_PINK_RGBH = (0xFE, 0x00, 0xA2)
TRIA_BLUE_RGBH = (0x00, 0xA2, 0xFE)
TRIA_YELLOW_RGBH = (0xFE, 0xDB, 0x00)
TRIA_GREEN_RGBH = (0x22, 0xB1, 0x4C)

APP_NAME = f"GTK GUI Node"

TRIA = r"""
████████╗██████╗ ██╗ █████╗ 
╚══██╔══╝██╔══██╗██║██╔══██╗
   ██║   ██████╔╝██║███████║
   ██║   ██╔══██╗██║██╔══██║
   ██║   ██║  ██║██║██║  ██║
   ╚═╝   ╚═╝  ╚═╝╚═╝╚═╝  ╚═╝
"""


def lerp(a, b, t):
    """Linear interpolation between two values"""
    return a + t * (b - a)


def inverse_lerp(a, b, v):
    """Inverse linear interpolation between two values"""
    return (v - a) / (b - a) if a != b else 0.0


def get_ema(x_cur, x_last, alpha=0.75):
    """
    Exponential moving average

    Args:
        x_cur: Current value
        x_last: Last value
        alpha: Smoothing factor

    Note:
        alpha is a misnomer. alpha = 1.0 is equivalent to no smoothing

    Ref:
        https://en.wikipedia.org/wiki/Exponential_smoothing

    """
    return alpha * x_cur + (1 - alpha) * x_last


def app_version():
    """Get the latest tag or commit hash if possible, unknown otherwise"""

    try:
        version = subprocess.check_output(
            ["git", "describe", "--tags", "--always"], text=True
        ).strip()
        date = subprocess.check_output(
            ["git", "log", "-1", "--format=%cd", "--date=short"], text=True
        ).strip()

        return f"{version} {date}"
    except subprocess.CalledProcessError:
        # Handle errors, such as not being in a Git repository
        return "unknown"


APP_HEADER = f"{APP_NAME} v({app_version()})"