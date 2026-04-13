#!/usr/bin/env python3

import math
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from tf2_ros import TransformBroadcaster


def _quat_multiply(a: Tuple[float, float, float, float], b: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _quat_conjugate(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    x, y, z, w = q
    return (-x, -y, -z, w)


def _rotate_vec_by_quat(v: Tuple[float, float, float], q: Tuple[float, float, float, float]) -> Tuple[float, float, float]:
    vx, vy, vz = v
    vq = (vx, vy, vz, 0.0)
    rq = _quat_multiply(_quat_multiply(q, vq), _quat_conjugate(q))
    return (rq[0], rq[1], rq[2])


def _lla_to_local_xy_m(lat_deg: float, lon_deg: float, ref_lat_deg: float, ref_lon_deg: float) -> Tuple[float, float]:
    # Equirectangular approximation for local map frame around reference GPS fix.
    earth_radius_m = 6378137.0
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    ref_lat = math.radians(ref_lat_deg)
    ref_lon = math.radians(ref_lon_deg)

    dlat = lat - ref_lat
    dlon = lon - ref_lon
    x_east = earth_radius_m * dlon * math.cos((lat + ref_lat) * 0.5)
    y_north = earth_radius_m * dlat
    return x_east, y_north


class MapOdomTfFromGps(Node):
    def __init__(self) -> None:
        super().__init__("map_odom_tf_from_gps")

        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.odom_child_frame = self.declare_parameter("odom_child_frame", "lidar_tc").value
        self.use_odom_msg_child_frame = bool(self.declare_parameter("use_odom_msg_child_frame", False).value)
        self.odom_topic = self.declare_parameter("odom_topic", "/novatel/oem7/odom").value
        self.gps_topic = self.declare_parameter("gps_topic", "/novatel/oem7/fix").value
        self.publish_rate_hz = float(self.declare_parameter("publish_rate_hz", 20.0).value)
        self.use_navsat_altitude = bool(self.declare_parameter("use_navsat_altitude", False).value)

        self._ref_lla: Optional[Tuple[float, float, float]] = None
        self._gps_pos_map: Optional[Tuple[float, float, float]] = None
        self._odom_pos: Optional[Tuple[float, float, float]] = None
        self._odom_quat: Optional[Tuple[float, float, float, float]] = None

        self._tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 20)
        self.create_subscription(NavSatFix, self.gps_topic, self._gps_cb, 20)

        period_s = 1.0 / max(self.publish_rate_hz, 1.0)
        self.create_timer(period_s, self._publish_tf)

        self.get_logger().info(
            f"Publishing dynamic {self.map_frame}->{self.odom_frame} TF from odom='{self.odom_topic}' gps='{self.gps_topic}'"
        )

    def _odom_cb(self, msg: Odometry) -> None:
        if msg.header.frame_id:
            self.odom_frame = msg.header.frame_id
        if self.use_odom_msg_child_frame and msg.child_frame_id:
            self.odom_child_frame = msg.child_frame_id
        self._odom_pos = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        )
        q = msg.pose.pose.orientation
        self._odom_quat = (q.x, q.y, q.z, q.w)

    def _gps_cb(self, msg: NavSatFix) -> None:
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return

        if self._ref_lla is None:
            ref_alt = 0.0 if math.isnan(msg.altitude) else msg.altitude
            self._ref_lla = (msg.latitude, msg.longitude, ref_alt)
            self.get_logger().info(
                f"GPS origin set at lat={msg.latitude:.8f}, lon={msg.longitude:.8f}, alt={ref_alt:.2f}"
            )

        assert self._ref_lla is not None
        x_east, y_north = _lla_to_local_xy_m(msg.latitude, msg.longitude, self._ref_lla[0], self._ref_lla[1])

        z = 0.0
        if self.use_navsat_altitude and (not math.isnan(msg.altitude)):
            z = msg.altitude - self._ref_lla[2]

        # map convention here: x=East, y=North
        self._gps_pos_map = (x_east, y_north, z)

    def _publish_tf(self) -> None:
        if self._gps_pos_map is None or self._odom_pos is None or self._odom_quat is None:
            return

        q_map_base = self._odom_quat
        q_odom_base = self._odom_quat
        q_base_odom = _quat_conjugate(q_odom_base)
        q_map_odom = _quat_multiply(q_map_base, q_base_odom)

        p_map_base = self._gps_pos_map
        p_odom_base = self._odom_pos
        p_odom_in_map = _rotate_vec_by_quat(p_odom_base, q_map_odom)
        p_map_odom = (
            p_map_base[0] - p_odom_in_map[0],
            p_map_base[1] - p_odom_in_map[1],
            p_map_base[2] - p_odom_in_map[2],
        )

        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = self.map_frame
        tf_msg.child_frame_id = self.odom_frame
        tf_msg.transform.translation.x = p_map_odom[0]
        tf_msg.transform.translation.y = p_map_odom[1]
        tf_msg.transform.translation.z = p_map_odom[2]
        tf_msg.transform.rotation.x = q_map_odom[0]
        tf_msg.transform.rotation.y = q_map_odom[1]
        tf_msg.transform.rotation.z = q_map_odom[2]
        tf_msg.transform.rotation.w = q_map_odom[3]

        self._tf_broadcaster.sendTransform(tf_msg)

        odom_child_tf = TransformStamped()
        odom_child_tf.header.stamp = tf_msg.header.stamp
        odom_child_tf.header.frame_id = self.odom_frame
        odom_child_tf.child_frame_id = self.odom_child_frame
        odom_child_tf.transform.translation.x = self._odom_pos[0]
        odom_child_tf.transform.translation.y = self._odom_pos[1]
        odom_child_tf.transform.translation.z = self._odom_pos[2]
        odom_child_tf.transform.rotation.x = self._odom_quat[0]
        odom_child_tf.transform.rotation.y = self._odom_quat[1]
        odom_child_tf.transform.rotation.z = self._odom_quat[2]
        odom_child_tf.transform.rotation.w = self._odom_quat[3]

        self._tf_broadcaster.sendTransform(odom_child_tf)


def main() -> None:
    rclpy.init()
    node = MapOdomTfFromGps()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
