import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _package_available(package_name):
    try:
        get_package_share_directory(package_name)
        return True
    except PackageNotFoundError:
        return False


def _resolve_camera_info_url(context, index):
    logger = get_logger("rear_cameras_only_bringup")
    config_dir = os.path.join(get_package_share_directory("vehicle_platform"), "config")

    url = LaunchConfiguration("rear_camera_%d_info_url" % index).perform(context).strip()
    if not url:
        frame_id = LaunchConfiguration("rear_camera_%d_frame_id" % index).perform(context).strip()
        candidate = os.path.join(config_dir, "%s_arena_camera_node.intrinsics.yaml" % frame_id)
        if frame_id and os.path.isfile(candidate):
            return "file://" + candidate
        logger.warning(
            "rear camera %d: no calibration configured; publishing uncalibrated CameraInfo" % index
        )
        return ""

    if "://" in url:
        return url

    path = url if os.path.isabs(url) else os.path.join(config_dir, url)
    if not os.path.isfile(path):
        logger.warning("rear camera %d: calibration file not found: %s" % (index, path))
    return "file://" + path


def _rear_camera_action(context, index):
    logger = get_logger("rear_cameras_only_bringup")
    if not _package_available("arena_camera_node"):
        logger.warning("Skipping rear camera_%d: package 'arena_camera_node' is not available" % index)
        return []

    enabled = LaunchConfiguration("enable_camera_rear_%d" % index).perform(context).lower()
    if enabled not in ["true", "1", "yes", "on"]:
        return []

    camera_name = "camera_rear_%d" % index
    return [
        Node(
            package="arena_camera_node",
            executable="start",
            namespace=camera_name,
            name="arena_camera_node",
            output="log",
            arguments=["--ros-args", "--log-level", "warn"],
            parameters=[
                {
                    "topic": "/%s/image" % camera_name,
                    "ip": LaunchConfiguration("rear_camera_%d_ip" % index),
                    "host_iface_ip": LaunchConfiguration("rear_camera_%d_host_iface_ip" % index),
                    "discovery_timeout_ms": LaunchConfiguration("camera_discovery_timeout_ms"),
                    "frame_rate": LaunchConfiguration("rear_camera_frame_rate_hz"),
                    "auto_exposure": LaunchConfiguration("rear_auto_exposure"),
                    "auto_exposure_min_us": LaunchConfiguration("rear_auto_exposure_min_us"),
                    "auto_exposure_max_us": LaunchConfiguration("rear_auto_exposure_max_us"),
                    "auto_exposure_target": LaunchConfiguration("rear_auto_exposure_target"),
                    "auto_gain": LaunchConfiguration("rear_auto_gain"),
                    "auto_gain_min": LaunchConfiguration("rear_auto_gain_min"),
                    "auto_gain_max": LaunchConfiguration("rear_auto_gain_max"),
                    "frame_id": LaunchConfiguration("rear_camera_%d_frame_id" % index),
                    "camera_info_url": _resolve_camera_info_url(context, index),
                    "use_ros_time": LaunchConfiguration("rear_camera_use_ros_time"),
                    "ptp_enable": LaunchConfiguration("rear_camera_ptp_enable"),
                    "packet_size": LaunchConfiguration("rear_camera_packet_size"),
                    "packet_delay": LaunchConfiguration("rear_camera_packet_delay"),
                    "stream_buffer_handling_mode": LaunchConfiguration(
                        "rear_camera_stream_buffer_handling_mode"
                    ),
                    "device_link_throughput_limit": LaunchConfiguration(
                        "rear_camera_link_throughput_limit"
                    ),
                    "trigger_mode": LaunchConfiguration("rear_trigger_mode"),
                    "trigger_source": LaunchConfiguration("rear_trigger_source"),
                    "trigger_selector": LaunchConfiguration("rear_trigger_selector"),
                    "trigger_activation": LaunchConfiguration("rear_trigger_activation"),
                }
            ],
        )
    ]


def _rear_image_proc_action(context, index):
    logger = get_logger("rear_cameras_only_bringup")
    if not _package_available("image_proc"):
        logger.warning("Skipping rear camera_%d image_proc: package 'image_proc' is not available" % index)
        return []

    enabled = LaunchConfiguration("enable_camera_rear_%d" % index).perform(context).lower()
    enable_rear_image_proc = LaunchConfiguration("enable_rear_image_proc").perform(context).lower()
    if enabled not in ["true", "1", "yes", "on"] or enable_rear_image_proc not in [
        "true",
        "1",
        "yes",
        "on",
    ]:
        return []

    return [
        Node(
            package="image_proc",
            executable="debayer_node",
            namespace="camera_rear_%d" % index,
            name="image_proc",
            output="screen",
            # 0 = bilinear; the default (3 = VNG) cannot keep up at 15 Hz on 3 MP frames
            parameters=[{"debayer": 0}],
            remappings=[("image_raw", "image")],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_discovery_timeout_ms",
                default_value="10000",
                description="Arena camera discovery timeout in milliseconds",
            ),
            DeclareLaunchArgument(
                "rear_camera_1_ip",
                default_value="192.168.50.10",
                description="Rear camera 1 IP",
            ),
            DeclareLaunchArgument(
                "rear_camera_2_ip",
                default_value="192.168.40.10",
                description="Rear camera 2 IP",
            ),
            DeclareLaunchArgument(
                "rear_camera_3_ip",
                default_value="192.168.30.10",
                description="Rear camera 3 IP",
            ),
            DeclareLaunchArgument(
                "rear_camera_4_ip",
                default_value="192.168.60.10",
                description="Rear camera 4 IP",
            ),
            DeclareLaunchArgument(
                "rear_camera_5_ip",
                default_value="192.168.70.10",
                description="Rear camera 5 IP",
            ),
            DeclareLaunchArgument(
                "rear_camera_1_host_iface_ip",
                default_value="192.168.50.100",
                description="Host NIC IP for rear camera 1 discovery",
            ),
            DeclareLaunchArgument(
                "rear_camera_2_host_iface_ip",
                default_value="192.168.40.100",
                description="Host NIC IP for rear camera 2 discovery",
            ),
            DeclareLaunchArgument(
                "rear_camera_3_host_iface_ip",
                default_value="192.168.30.100",
                description="Host NIC IP for rear camera 3 discovery",
            ),
            DeclareLaunchArgument(
                "rear_camera_4_host_iface_ip",
                default_value="192.168.60.100",
                description="Host NIC IP for rear camera 4 discovery",
            ),
            DeclareLaunchArgument(
                "rear_camera_5_host_iface_ip",
                default_value="192.168.70.100",
                description="Host NIC IP for rear camera 5 discovery",
            ),
            DeclareLaunchArgument(
                "rear_camera_frame_rate_hz",
                default_value="15.0",
                description="Target rear camera frame rate in Hz",
            ),
            DeclareLaunchArgument(
                "rear_auto_exposure",
                default_value="true",
                description="Enable rear camera auto exposure",
            ),
            DeclareLaunchArgument(
                "rear_auto_exposure_min_us",
                default_value="50.0",
                description="Rear camera minimum exposure in microseconds",
            ),
            DeclareLaunchArgument(
                "rear_auto_exposure_max_us",
                default_value="10000.0",
                description="Rear camera maximum exposure in microseconds",
            ),
            DeclareLaunchArgument(
                "rear_auto_exposure_target",
                default_value="95.0",
                description="Rear camera target brightness (0-255)",
            ),
            DeclareLaunchArgument(
                "rear_auto_gain",
                default_value="true",
                description="Enable rear camera auto gain",
            ),
            DeclareLaunchArgument(
                "rear_auto_gain_min",
                default_value="-1.0",
                description="Rear camera minimum auto gain in dB",
            ),
            DeclareLaunchArgument(
                "rear_auto_gain_max",
                default_value="10.0",
                description="Rear camera maximum auto gain in dB",
            ),
            DeclareLaunchArgument(
                "rear_camera_use_ros_time",
                default_value="true",
                description="Use ROS time for rear camera stamps",
            ),
            DeclareLaunchArgument(
                "rear_camera_ptp_enable",
                default_value="false",
                description="Enable IEEE-1588 PTP on rear cameras",
            ),
            DeclareLaunchArgument(
                "rear_camera_1_frame_id",
                default_value="camera_rear_1",
                description="TF frame for rear camera 1",
            ),
            DeclareLaunchArgument(
                "rear_camera_2_frame_id",
                default_value="camera_rear_2",
                description="TF frame for rear camera 2",
            ),
            DeclareLaunchArgument(
                "rear_camera_3_frame_id",
                default_value="camera_rear_3",
                description="TF frame for rear camera 3",
            ),
            DeclareLaunchArgument(
                "rear_camera_4_frame_id",
                default_value="camera_rear_4",
                description="TF frame for rear camera 4",
            ),
            DeclareLaunchArgument(
                "rear_camera_5_frame_id",
                default_value="camera_rear_5",
                description="TF frame for rear camera 5",
            ),
            DeclareLaunchArgument(
                "rear_camera_1_info_url",
                default_value="",
                description="Calibration URL/path for rear camera 1",
            ),
            DeclareLaunchArgument(
                "rear_camera_2_info_url",
                default_value="",
                description="Calibration URL/path for rear camera 2",
            ),
            DeclareLaunchArgument(
                "rear_camera_3_info_url",
                default_value="",
                description="Calibration URL/path for rear camera 3",
            ),
            DeclareLaunchArgument(
                "rear_camera_4_info_url",
                default_value="",
                description="Calibration URL/path for rear camera 4",
            ),
            DeclareLaunchArgument(
                "rear_camera_5_info_url",
                default_value="",
                description="Calibration URL/path for rear camera 5",
            ),
            DeclareLaunchArgument(
                "rear_camera_packet_size",
                default_value="9000",
                description="GigE stream packet size for rear cameras",
            ),
            DeclareLaunchArgument(
                "rear_camera_packet_delay",
                default_value="-1",
                description="Inter-packet delay for rear cameras",
            ),
            DeclareLaunchArgument(
                "rear_camera_stream_buffer_handling_mode",
                default_value="",
                description="Rear camera buffer policy",
            ),
            DeclareLaunchArgument(
                "rear_camera_link_throughput_limit",
                default_value="-1",
                description="Rear camera throughput limit in bytes/s",
            ),
            DeclareLaunchArgument(
                "rear_trigger_mode",
                default_value="false",
                description="Enable rear camera trigger mode",
            ),
            DeclareLaunchArgument(
                "rear_trigger_source",
                default_value="Freerun",
                description="Rear camera trigger source",
            ),
            DeclareLaunchArgument(
                "rear_trigger_selector",
                default_value="FrameStart",
                description="Rear camera trigger selector",
            ),
            DeclareLaunchArgument(
                "rear_trigger_activation",
                default_value="FixedRate",
                description="Rear camera trigger activation",
            ),
            DeclareLaunchArgument(
                "enable_camera_rear_1",
                default_value="true",
                description="Enable rear camera 1",
            ),
            DeclareLaunchArgument(
                "enable_camera_rear_2",
                default_value="true",
                description="Enable rear camera 2",
            ),
            DeclareLaunchArgument(
                "enable_camera_rear_3",
                default_value="true",
                description="Enable rear camera 3",
            ),
            DeclareLaunchArgument(
                "enable_camera_rear_4",
                default_value="true",
                description="Enable rear camera 4",
            ),
            DeclareLaunchArgument(
                "enable_camera_rear_5",
                default_value="true",
                description="Enable rear camera 5",
            ),
            DeclareLaunchArgument(
                "enable_rear_image_proc",
                default_value="true",
                description="Enable image_proc debayer for rear camera topics",
            ),
            OpaqueFunction(function=lambda ctx: _rear_camera_action(ctx, 1)),
            OpaqueFunction(function=lambda ctx: _rear_camera_action(ctx, 2)),
            OpaqueFunction(function=lambda ctx: _rear_camera_action(ctx, 3)),
            OpaqueFunction(function=lambda ctx: _rear_camera_action(ctx, 4)),
            OpaqueFunction(function=lambda ctx: _rear_camera_action(ctx, 5)),
            OpaqueFunction(function=lambda ctx: _rear_image_proc_action(ctx, 1)),
            OpaqueFunction(function=lambda ctx: _rear_image_proc_action(ctx, 2)),
            OpaqueFunction(function=lambda ctx: _rear_image_proc_action(ctx, 3)),
            OpaqueFunction(function=lambda ctx: _rear_image_proc_action(ctx, 4)),
            OpaqueFunction(function=lambda ctx: _rear_image_proc_action(ctx, 5)),
        ]
    )