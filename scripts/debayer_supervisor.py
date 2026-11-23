#!/usr/bin/env python3
"""Run image_proc debayer nodes and restart any that hang.

rmw_zenoh can deadlock a debayer node (main thread in rmw_publish and a zenoh
rx thread waiting on each other's locks). The process stays alive, so launch
never respawns it. This supervisor owns the debayer processes and treats one as
hung when its camera still publishes camera_info, its image_color/compressed
output has been silent for --timeout seconds, and it used no CPU in that time.
A hung node gets a gdb backtrace saved to --bt-dir and is restarted.

  debayer_supervisor.py camera_fl camera_fr camera_rear_1 ...
"""

import argparse
import ctypes
import os
import signal
import subprocess
import sys
import time
from datetime import datetime

import rclpy
from ament_index_python.packages import get_package_prefix
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import CameraInfo, CompressedImage

PR_SET_PDEATHSIG = 1
PR_SET_PTRACER = 0x59616D61
PR_SET_PTRACER_ANY = ctypes.c_ulong(0xFFFFFFFFFFFFFFFF)
CLK_TCK = os.sysconf("SC_CLK_TCK")


def _child_setup():
    libc = ctypes.CDLL(None)
    # Let gdb (a sibling, not an ancestor) attach without sudo under
    # ptrace_scope=1, and take the child down with us if we die.
    libc.prctl(PR_SET_PTRACER, PR_SET_PTRACER_ANY, 0, 0, 0)
    libc.prctl(PR_SET_PDEATHSIG, signal.SIGTERM, 0, 0, 0)


def _cpu_ticks(pid):
    try:
        with open(f"/proc/{pid}/stat") as f:
            fields = f.read().rsplit(")", 1)[1].split()
        return int(fields[11]) + int(fields[12])
    except (OSError, IndexError, ValueError):
        return None


class Camera:
    def __init__(self, name, args, log):
        self.name = name
        self.args = args
        self.log = log
        self.proc = None
        self.started = 0.0
        self.last_out = None
        self.last_info = None
        self.cpu_samples = []
        self.restarts = 0

    def start(self):
        exe = os.path.join(get_package_prefix("image_proc"), "lib", "image_proc", "debayer_node")
        cmd = [exe, "--ros-args", "-r", "__node:=image_proc", "-r", f"__ns:=/{self.name}",
               "-r", "image_raw:=image", "-p", f"debayer:={self.args.debayer}"]
        self.proc = subprocess.Popen(cmd, preexec_fn=_child_setup)
        self.started = time.monotonic()
        self.last_out = None
        self.cpu_samples = []
        self.log(f"{self.name}: started debayer pid {self.proc.pid}")

    def kill(self):
        if self.proc and self.proc.poll() is None:
            self.proc.kill()
            self.proc.wait()

    def backtrace(self):
        os.makedirs(self.args.bt_dir, exist_ok=True)
        path = os.path.join(self.args.bt_dir, f"{self.name}_{datetime.now():%Y%m%d_%H%M%S}.txt")
        try:
            r = subprocess.run(["gdb", "-p", str(self.proc.pid), "-batch", "-ex", "thread apply all bt"],
                               capture_output=True, text=True, timeout=90)
            with open(path, "w") as f:
                f.write(r.stdout + r.stderr)
            return path
        except (OSError, subprocess.TimeoutExpired) as e:
            return f"backtrace failed: {e}"

    def check(self, now):
        if self.proc.poll() is not None:
            self.log(f"{self.name}: debayer exited with code {self.proc.returncode}, restarting")
            self.restarts += 1
            self.start()
            return
        ticks = _cpu_ticks(self.proc.pid)
        if ticks is not None:
            self.cpu_samples.append((now, ticks))
        self.cpu_samples = [s for s in self.cpu_samples if s[0] >= now - self.args.timeout - 1]

        if now - self.started < self.args.grace:
            return
        camera_alive = self.last_info is not None and now - self.last_info < self.args.timeout
        out_since = self.last_out if self.last_out is not None else self.started
        if not camera_alive or now - out_since < self.args.timeout:
            return
        span = self.cpu_samples[-1][0] - self.cpu_samples[0][0] if len(self.cpu_samples) > 1 else 0
        if span < self.args.timeout - 1:
            return
        cpu_s = (self.cpu_samples[-1][1] - self.cpu_samples[0][1]) / CLK_TCK
        if cpu_s > 0.05:
            return  # busy but not publishing: not the deadlock signature, leave it alone
        self.log(f"{self.name}: HUNG (camera publishing, no output for {now - out_since:.0f}s, "
                 f"{cpu_s:.2f}s CPU) - saving backtrace")
        self.log(f"{self.name}: backtrace -> {self.backtrace()}")
        self.kill()
        self.restarts += 1
        self.start()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cameras", nargs="+", help="camera namespaces, e.g. camera_fr")
    ap.add_argument("--debayer", type=int, default=0, help="debayer algorithm (0 = bilinear)")
    ap.add_argument("--timeout", type=float, default=5.0, help="seconds of silent output before hang check")
    ap.add_argument("--grace", type=float, default=15.0, help="seconds after (re)start before checking")
    ap.add_argument("--bt-dir", default=os.path.expanduser("~/.ros/log/debayer_hangs"))
    args = ap.parse_args(remove_ros_args(sys.argv)[1:])  # drop --ros-args added by launch

    def log(msg):
        print(f"[{datetime.now():%Y-%m-%d %H:%M:%S}] [debayer_supervisor] {msg}", flush=True)

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node("debayer_supervisor")
    # Best effort: watching must never back-pressure the cameras.
    qos = QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.BEST_EFFORT)
    cams = [Camera(name, args, log) for name in args.cameras]
    for cam in cams:
        def on_out(_, cam=cam):
            cam.last_out = time.monotonic()

        def on_info(_, cam=cam):
            cam.last_info = time.monotonic()

        # Keeping a subscriber on the output also keeps the lazy debayer active.
        node.create_subscription(CompressedImage, f"/{cam.name}/image_color/compressed", on_out, qos, raw=True)
        node.create_subscription(CameraInfo, f"/{cam.name}/camera_info", on_info, qos, raw=True)
        cam.start()

    stop = [False]

    def on_signal(*_):
        stop[0] = True

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    next_check = time.monotonic() + 1.0
    try:
        while not stop[0]:
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            if now >= next_check:
                next_check = now + 1.0
                for cam in cams:
                    cam.check(now)
    finally:
        for cam in cams:
            if cam.proc and cam.proc.poll() is None:
                cam.proc.send_signal(signal.SIGINT)
        deadline = time.monotonic() + 5
        for cam in cams:
            try:
                cam.proc.wait(timeout=max(0.1, deadline - time.monotonic()))
            except subprocess.TimeoutExpired:
                cam.kill()
        log("stopped; restarts: " + ", ".join(f"{c.name}={c.restarts}" for c in cams))
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
