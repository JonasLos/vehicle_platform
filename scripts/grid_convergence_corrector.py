#!/usr/bin/env python3
"""
Grid-convergence corrector for NovAtel UTM odometry.

The NovAtel driver publishes the INS azimuth referenced to TRUE north, while the
odometry position (and therefore any course computed from it, and every path CSV
recorded from it) is in UTM, whose grid north is rotated from true north by the
grid convergence angle:

    gamma = atan( tan(lon - lon_cm) * sin(lat) )

At the RELLIS runway (lat 30.60 N, lon -96.48, UTM zone 14 CM = -99 deg) gamma is
+1.286 deg. Validated against rosbag2_2026_07_08 bags: the raw quat-vs-course
offset of -1.2..-1.4 deg equals -gamma at each test location, so the corrected
yaw is raw_yaw + gamma.

Deploy: remap the driver's odometry output to /novatel/oem7/odom_raw and run
this node, which republishes the corrected /novatel/oem7/odom. Downstream
consumers are unchanged. Body-frame twist is passed through untouched (it is
frame-consistent once the heading is corrected).
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry


def utm_to_latlon(easting, northing, central_meridian_deg):
    """Approximate inverse UTM (northern hemisphere). Accurate to ~0.02 deg,
    which bounds the convergence error below 0.01 deg."""
    k0 = 0.9996
    lat = northing / (111132.0 * k0)
    for _ in range(3):
        lat = northing / (k0 * (111132.92 - 559.82 * math.cos(2.0 * math.radians(lat))))
    dlon = (easting - 500000.0) / (k0 * 111320.0 * math.cos(math.radians(lat)))
    return lat, central_meridian_deg + dlon


def grid_convergence_deg(easting, northing, central_meridian_deg):
    lat, lon = utm_to_latlon(easting, northing, central_meridian_deg)
    return math.degrees(math.atan(
        math.tan(math.radians(lon - central_meridian_deg)) * math.sin(math.radians(lat))
    ))


def rotate_quat_about_z(q, angle_rad):
    """Pre-multiply q by a world-frame rotation about +Z."""
    half = angle_rad / 2.0
    rw, rz = math.cos(half), math.sin(half)
    # (rw + rz*k) * (q.w + q.x*i + q.y*j + q.z*k)
    return (
        rw * q.x - rz * q.y,   # x
        rw * q.y + rz * q.x,   # y
        rw * q.z + rz * q.w,   # z
        rw * q.w - rz * q.z,   # w
    )


def quat_to_yaw_deg(q):
    return math.degrees(math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    ))


def wrapped_delta_deg(a, b):
    """Shortest signed difference a-b, wrapped to [-180, 180]."""
    return (a - b + 180.0) % 360.0 - 180.0


def compass_bearing_deg(enu_yaw_deg):
    """0 deg = North, clockwise-positive, from a REP103 ENU yaw (0 deg = East, CCW+)."""
    return (90.0 - enu_yaw_deg) % 360.0


class GridConvergenceCorrector(Node):
    def __init__(self):
        super().__init__("grid_convergence_corrector")
        self.declare_parameter("central_meridian_deg", -99.0)  # UTM zone 14
        self.declare_parameter("in_topic", "/novatel/oem7/odom")
        self.declare_parameter("out_topic", "/novatel/oem7/odom_grid")
        self.cm = self.get_parameter("central_meridian_deg").value

        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(
            Odometry, self.get_parameter("out_topic").value, qos)
        self.create_subscription(
            Odometry, self.get_parameter("in_topic").value, self.callback, qos)
        self._prev_raw_yaw = None
        self._prev_corrected_yaw = None

    def callback(self, msg):
        raw_yaw = quat_to_yaw_deg(msg.pose.pose.orientation)

        gamma = grid_convergence_deg(
            msg.pose.pose.position.x, msg.pose.pose.position.y, self.cm)
        q = msg.pose.pose.orientation
        x, y, z, w = rotate_quat_about_z(q, math.radians(gamma))
        msg.pose.pose.orientation.x = x
        msg.pose.pose.orientation.y = y
        msg.pose.pose.orientation.z = z
        msg.pose.pose.orientation.w = w

        corrected_yaw = quat_to_yaw_deg(msg.pose.pose.orientation)

        d_raw = 0.0 if self._prev_raw_yaw is None else wrapped_delta_deg(raw_yaw, self._prev_raw_yaw)
        d_corr = 0.0 if self._prev_corrected_yaw is None else wrapped_delta_deg(corrected_yaw, self._prev_corrected_yaw)

        raw_compass = compass_bearing_deg(raw_yaw)
        corrected_compass = compass_bearing_deg(corrected_yaw)

        line = (f"yaw[ENU] raw={raw_yaw:+7.2f} (d={d_raw:+6.2f})  "
                f"corrected={corrected_yaw:+7.2f} (d={d_corr:+6.2f})  "
                f"| compass[0=N,CW] raw={raw_compass:6.2f}  corrected={corrected_compass:6.2f}  "
                f"gamma={gamma:+.3f}")
        #self.get_logger().info(line, throttle_duration_sec=0.2)
        #print(line)

        if abs(d_raw) > 90.0:
            self.get_logger().warn(f"discontinuity in raw yaw: jumped {d_raw:+.2f} deg")
        if abs(d_corr) > 90.0:
            self.get_logger().warn(f"discontinuity in corrected yaw: jumped {d_corr:+.2f} deg")

        self._prev_raw_yaw = raw_yaw
        self._prev_corrected_yaw = corrected_yaw

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GridConvergenceCorrector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
