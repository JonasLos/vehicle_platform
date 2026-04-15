#!/usr/bin/env python3

import contextlib
import importlib
import io
import json
import math
import sys
from pathlib import Path
from typing import List, Optional, Tuple

import rclpy
from carma_driver_msgs.msg import ByteArray
from geometry_msgs.msg import TransformStamped
from j2735_v2x_msgs.msg import MapData
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from tf2_ros import TransformBroadcaster


VALID_V2X_IDS = {0x12, 0x13, 0x14, 0x1D, 0x1E, 0x1F, 0x20, 0x29}


def _try_load_j2735_decoder_module():
    try:
        return importlib.import_module("j2735_202409")
    except ImportError:
        pass

    current = Path(__file__).resolve()
    roots = [current.parent]
    roots.extend(current.parents)
    for root in roots:
        venv_lib = root / ".venv" / "lib"
        if not venv_lib.is_dir():
            continue
        for py_dir in sorted(venv_lib.glob("python*")):
            site_packages = py_dir / "site-packages"
            if not site_packages.is_dir():
                continue
            site_packages_str = str(site_packages)
            if site_packages_str not in sys.path:
                sys.path.insert(0, site_packages_str)
            try:
                return importlib.import_module("j2735_202409")
            except ImportError:
                continue

    raise ImportError("No module named 'j2735_202409'")


def _extract_framed_candidates(data: bytes) -> List[bytes]:
    candidates: List[bytes] = []

    for idx in range(len(data) - 3):
        if data[idx] != 0x00:
            continue
        msg_id = data[idx + 1]
        msg_len = data[idx + 2]
        if msg_id in VALID_V2X_IDS and msg_len > 0:
            end = idx + 3 + msg_len
            if end <= len(data):
                candidates.append(data[idx:end])

    for idx in range(len(data) - 2):
        msg_id = data[idx]
        msg_len = data[idx + 1]
        if msg_id in VALID_V2X_IDS and msg_len > 0:
            end = idx + 2 + msg_len
            if end <= len(data):
                candidates.append(data[idx:end])

    for msg_id in (b"\x00\x12", b"\x00\x13", b"\x00\x14", b"\x00\x1d", b"\x00\x1e", b"\x00\x1f", b"\x00\x20", b"\x00\x29"):
        idx = data.find(msg_id)
        if idx != -1:
            candidates.append(data[idx:])

    return candidates


def _try_decode(payload: bytes, message_frame) -> Optional[dict]:
    try:
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            message_frame.from_uper(payload)
            decoded_json = message_frame.to_jer()
        parsed = json.loads(decoded_json)
    except Exception:
        return None

    value = parsed.get("value")
    if not isinstance(value, (dict, list)):
        return None

    return parsed


def _deep_scan_for_decode(data: bytes, message_frame) -> Optional[dict]:
    max_scan_len = min(260, len(data))
    for start in range(len(data)):
        max_end = min(len(data), start + max_scan_len)
        for end in range(start + 18, max_end + 1):
            decoded = _try_decode(data[start:end], message_frame)
            if decoded is not None:
                return decoded
    return None


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
        self.use_fixed_reference = bool(self.declare_parameter("use_fixed_reference", False).value)
        self.fixed_reference_lat_deg = float(self.declare_parameter("fixed_reference_lat_deg", 0.0).value)
        self.fixed_reference_lon_deg = float(self.declare_parameter("fixed_reference_lon_deg", 0.0).value)
        self.fixed_reference_alt_m = float(self.declare_parameter("fixed_reference_alt_m", 0.0).value)
        self.anchor_mode = str(self.declare_parameter("anchor_mode", "gps_fallback").value).strip().lower()
        self.map_anchor_topic = self.declare_parameter("map_anchor_topic", "/message/incoming_map").value
        self.inbound_map_anchor_topic = self.declare_parameter("inbound_map_anchor_topic", "/comms/inbound_binary_msg").value
        self.inbound_map_anchor_enable_deep_scan = bool(
            self.declare_parameter("inbound_map_anchor_enable_deep_scan", True).value
        )
        self.allow_anchor_relock = bool(self.declare_parameter("allow_anchor_relock", False).value)
        self.reanchor_distance_m = float(self.declare_parameter("reanchor_distance_m", 150.0).value)
        self.publish_direct_map_to_odom_child_tf = bool(
            self.declare_parameter("publish_direct_map_to_odom_child_tf", True).value
        )
        self.direct_map_to_odom_child_use_odom_z = bool(
            self.declare_parameter("direct_map_to_odom_child_use_odom_z", False).value
        )
        self.publish_odom_to_odom_child_tf = bool(
            self.declare_parameter("publish_odom_to_odom_child_tf", False).value
        )

        self._ref_lla: Optional[Tuple[float, float, float]] = None
        self._gps_pos_map: Optional[Tuple[float, float, float]] = None
        self._odom_pos: Optional[Tuple[float, float, float]] = None
        self._odom_quat: Optional[Tuple[float, float, float, float]] = None
        self._anchor_source: str = "none"
        self._message_frame = None

        self._tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 20)
        self.create_subscription(NavSatFix, self.gps_topic, self._gps_cb, 20)
        self._map_sub = None
        self._inbound_map_sub = None
        if not self.use_fixed_reference and self.anchor_mode in ("map_only", "map_then_lock"):
            self._map_sub = self.create_subscription(MapData, self.map_anchor_topic, self._map_cb, 10)
            if self.inbound_map_anchor_topic:
                try:
                    j2735_202409 = _try_load_j2735_decoder_module()
                    self._message_frame = j2735_202409.MessageFrame.MessageFrame
                    self._inbound_map_sub = self.create_subscription(
                        ByteArray,
                        self.inbound_map_anchor_topic,
                        self._inbound_map_cb,
                        50,
                    )
                except ImportError as exc:
                    self.get_logger().warning(
                        "Raw inbound MAP anchoring disabled; missing decoder module j2735_202409: %s" % exc
                    )

        period_s = 1.0 / max(self.publish_rate_hz, 1.0)
        self.create_timer(period_s, self._publish_tf)

        self.get_logger().info(
            f"Publishing dynamic {self.map_frame}->{self.odom_frame} TF from odom='{self.odom_topic}' gps='{self.gps_topic}'"
        )
        self.get_logger().info(
            "Anchor mode=%s map_anchor_topic=%s inbound_map_anchor_topic=%s allow_anchor_relock=%s reanchor_distance_m=%.1f"
            % (
                self.anchor_mode,
                self.map_anchor_topic,
                self.inbound_map_anchor_topic,
                str(self.allow_anchor_relock),
                self.reanchor_distance_m,
            )
        )
        self.get_logger().info(
            "Direct map->%s TF from odom=%s (use_odom_z=%s), odom->%s TF=%s"
            % (
                self.odom_child_frame,
                str(self.publish_direct_map_to_odom_child_tf),
                str(self.direct_map_to_odom_child_use_odom_z),
                self.odom_child_frame,
                str(self.publish_odom_to_odom_child_tf),
            )
        )
        if self.use_fixed_reference:
            self._ref_lla = (
                self.fixed_reference_lat_deg,
                self.fixed_reference_lon_deg,
                self.fixed_reference_alt_m,
            )
            self._anchor_source = "fixed_reference"
            self.get_logger().info(
                "Using fixed map GPS origin lat=%.8f lon=%.8f alt=%.2f"
                % (self.fixed_reference_lat_deg, self.fixed_reference_lon_deg, self.fixed_reference_alt_m)
            )

    def _set_anchor_from_refs(self, refs: List[Tuple[float, float]], source_name: str) -> None:
        if not refs:
            return

        lat = sum(v[0] for v in refs) / float(len(refs))
        lon = sum(v[1] for v in refs) / float(len(refs))

        if self._ref_lla is None:
            self._ref_lla = (lat, lon, self.fixed_reference_alt_m)
            self._anchor_source = source_name
            self.get_logger().info(
                "%s anchor set at lat=%.8f lon=%.8f (%d intersections)"
                % (source_name, lat, lon, len(refs))
            )
            return

        # In map_then_lock mode, allow one-time handoff from provisional GPS
        # origin to the first MAP-derived origin, then keep it locked.
        if self.anchor_mode == "map_then_lock" and self._anchor_source == "gps_first_fix":
            self._ref_lla = (lat, lon, self.fixed_reference_alt_m)
            self._anchor_source = source_name
            self.get_logger().info(
                "%s anchor replaced provisional GPS anchor at lat=%.8f lon=%.8f"
                % (source_name, lat, lon)
            )
            return

        dx, dy = _lla_to_local_xy_m(lat, lon, self._ref_lla[0], self._ref_lla[1])
        dist_m = math.hypot(dx, dy)
        if dist_m < self.reanchor_distance_m:
            return

        if not self.allow_anchor_relock:
            self.get_logger().warn(
                "Ignoring %s anchor shift of %.1f m (relock disabled)." % (source_name, dist_m)
            )
            return

        self._ref_lla = (lat, lon, self.fixed_reference_alt_m)
        self._anchor_source = "%s_relocked" % source_name
        self.get_logger().warn(
            "%s anchor relocked to lat=%.8f lon=%.8f (shift %.1f m)" % (source_name, lat, lon, dist_m)
        )

    def _map_cb(self, msg: MapData) -> None:
        if not msg.intersections_exists or not msg.intersections:
            return

        refs = []
        for inter in msg.intersections:
            ref = inter.ref_point
            refs.append((float(ref.latitude) * 1e-7, float(ref.longitude) * 1e-7))

        self._set_anchor_from_refs(refs, "decoded_map")

    def _decode_inbound_map(self, payload: bytes) -> Optional[dict]:
        if self._message_frame is None:
            return None

        for candidate in _extract_framed_candidates(payload):
            decoded = _try_decode(candidate, self._message_frame)
            if decoded is not None:
                return decoded

        if self.inbound_map_anchor_enable_deep_scan:
            return _deep_scan_for_decode(payload, self._message_frame)

        return None

    def _inbound_map_cb(self, msg: ByteArray) -> None:
        payload = bytes(msg.content)
        if not payload:
            return

        driver_type = (msg.message_type or "").strip().upper()
        if driver_type and driver_type != "MAP":
            return

        decoded = self._decode_inbound_map(payload)
        if decoded is None:
            return

        message_id = decoded.get("messageId")
        if driver_type != "MAP" and message_id != 18:
            return

        value = decoded.get("value")
        if not isinstance(value, dict):
            return

        intersections = value.get("intersections")
        if not isinstance(intersections, list):
            return

        refs: List[Tuple[float, float]] = []
        for inter in intersections:
            if not isinstance(inter, dict):
                continue
            ref = inter.get("refPoint", {})
            if not isinstance(ref, dict):
                continue
            ref_lat_raw = ref.get("lat")
            ref_lon_raw = ref.get("long")
            if not isinstance(ref_lon_raw, int):
                ref_lon_raw = ref.get("lon")
            if not isinstance(ref_lat_raw, int) or not isinstance(ref_lon_raw, int):
                continue
            refs.append((ref_lat_raw * 1e-7, ref_lon_raw * 1e-7))

        self._set_anchor_from_refs(refs, "inbound_map")

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
            if self.anchor_mode == "map_only":
                # MAP-gated mode: do not produce map-frame TF until a MAP anchor is known.
                return

            ref_alt = 0.0 if math.isnan(msg.altitude) else msg.altitude
            self._ref_lla = (msg.latitude, msg.longitude, ref_alt)
            self._anchor_source = "gps_first_fix"
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
        if self._odom_pos is None or self._odom_quat is None:
            return

        stamp = self.get_clock().now().to_msg()

        if self.publish_direct_map_to_odom_child_tf and self.odom_child_frame != self.map_frame:
            direct_tf = TransformStamped()
            direct_tf.header.stamp = stamp
            direct_tf.header.frame_id = self.map_frame
            direct_tf.child_frame_id = self.odom_child_frame
            direct_tf.transform.translation.x = self._odom_pos[0]
            direct_tf.transform.translation.y = self._odom_pos[1]
            # Keep direct map->odom_child on ground plane by default so map markers at z=0
            # align vertically with vehicle unless explicit odom Z is requested.
            direct_tf.transform.translation.z = self._odom_pos[2] if self.direct_map_to_odom_child_use_odom_z else 0.0
            direct_tf.transform.rotation.x = self._odom_quat[0]
            direct_tf.transform.rotation.y = self._odom_quat[1]
            direct_tf.transform.rotation.z = self._odom_quat[2]
            direct_tf.transform.rotation.w = self._odom_quat[3]
            self._tf_broadcaster.sendTransform(direct_tf)

        if self._gps_pos_map is None:
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
        tf_msg.header.stamp = stamp
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

        if not self.publish_odom_to_odom_child_tf:
            return

        odom_child_tf = TransformStamped()
        odom_child_tf.header.stamp = stamp
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
