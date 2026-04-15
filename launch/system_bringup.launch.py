import os
import yaml

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.logging import get_logger
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def _load_ros_params(file_path, node_key):
    with open(file_path, "r", encoding="utf-8") as handle:
        loaded = yaml.safe_load(handle)
    return loaded[node_key]["ros__parameters"]


def _velodyne_group(namespace, frame_id, driver_params_file, transform_cfg_name, calibration_name):
    pointcloud_share = get_package_share_directory("velodyne_pointcloud")
    laserscan_share = get_package_share_directory("velodyne_laserscan")

    driver_params = _load_ros_params(driver_params_file, "velodyne_driver_node")
    driver_params["frame_id"] = frame_id

    transform_cfg_file = os.path.join(pointcloud_share, "config", transform_cfg_name)
    transform_params = _load_ros_params(transform_cfg_file, "velodyne_transform_node")
    transform_params["calibration"] = os.path.join(pointcloud_share, "params", calibration_name)

    laserscan_params_file = os.path.join(
        laserscan_share, "config", "default-velodyne_laserscan_node-params.yaml"
    )

    return GroupAction(
        actions=[
            PushRosNamespace(namespace),
            Node(
                package="velodyne_driver",
                executable="velodyne_driver_node",
                output="screen",
                parameters=[driver_params],
            ),
            Node(
                package="velodyne_pointcloud",
                executable="velodyne_transform_node",
                output="screen",
                parameters=[transform_params],
            ),
            Node(
                package="velodyne_laserscan",
                executable="velodyne_laserscan_node",
                output="screen",
                parameters=[laserscan_params_file],
            ),
        ]
    )


def _front_camera_fl_action(context):
    logger = get_logger("vehicle_platform_bringup")
    if not _package_available("avt_vimba_camera"):
        logger.warning("Skipping front camera_fl: package 'avt_vimba_camera' is not available")
        return []
    
    enable_camera_fl = LaunchConfiguration("enable_camera_fl").perform(context).lower()
    if enable_camera_fl not in ["true", "1", "yes", "on"]:
        return []

    avt_share = get_package_share_directory("avt_vimba_camera")
    mako_launch = os.path.join(avt_share, "launch", "Mako_G-319.launch.xml")

    return [
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(mako_launch),
            launch_arguments={
                "name": "camera_fl",
                "frame_id": "camera_fl",
                "ip": LaunchConfiguration("front_camera_fl_ip").perform(context),
                "camera_info_url": LaunchConfiguration("front_camera_fl_info_url").perform(context),
                "params_file": LaunchConfiguration("front_camera_params_file").perform(context),
                "image_proc": "false",
            }.items(),
        )
    ]


def _front_camera_fr_action(context):
    logger = get_logger("vehicle_platform_bringup")
    if not _package_available("avt_vimba_camera"):
        logger.warning("Skipping front camera_fr: package 'avt_vimba_camera' is not available")
        return []

    enable_camera_fr = LaunchConfiguration("enable_camera_fr").perform(context).lower()
    if enable_camera_fr not in ["true", "1", "yes", "on"]:
        return []

    avt_share = get_package_share_directory("avt_vimba_camera")
    mako_launch = os.path.join(avt_share, "launch", "Mako_G-319.launch.xml")

    return [
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(mako_launch),
            launch_arguments={
                "name": "camera_fr",
                "frame_id": "camera_fr",
                "ip": LaunchConfiguration("front_camera_fr_ip").perform(context),
                "params_file": LaunchConfiguration("front_camera_params_file").perform(context),
                "image_proc": "false",
            }.items(),
        )
    ]


def _front_image_proc_fl_action(context):
    logger = get_logger("vehicle_platform_bringup")
    if not _package_available("image_proc"):
        logger.warning("Skipping front camera_fl image_proc: package 'image_proc' is not available")
        return []

    enable_camera = LaunchConfiguration("enable_camera_fl").perform(context).lower()
    enable_front_image_proc = LaunchConfiguration("enable_front_image_proc").perform(context).lower()
    if enable_camera not in ["true", "1", "yes", "on"] or enable_front_image_proc not in [
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
            namespace="camera_fl",
            name="image_proc",
            output="screen",
            remappings=[("image_raw", "image")],
        )
    ]


def _front_image_proc_fr_action(context):
    logger = get_logger("vehicle_platform_bringup")
    if not _package_available("image_proc"):
        logger.warning("Skipping front camera_fr image_proc: package 'image_proc' is not available")
        return []

    enable_camera = LaunchConfiguration("enable_camera_fr").perform(context).lower()
    enable_front_image_proc = LaunchConfiguration("enable_front_image_proc").perform(context).lower()
    if enable_camera not in ["true", "1", "yes", "on"] or enable_front_image_proc not in [
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
            namespace="camera_fr",
            name="image_proc",
            output="screen",
            remappings=[("image_raw", "image")],
        )
    ]


def _rear_image_proc_1_action(context):
    logger = get_logger("vehicle_platform_bringup")
    if not _package_available("image_proc"):
        logger.warning("Skipping rear camera_1 image_proc: package 'image_proc' is not available")
        return []

    enable_camera = LaunchConfiguration("enable_camera_rear_1").perform(context).lower()
    enable_rear_image_proc = LaunchConfiguration("enable_rear_image_proc").perform(context).lower()
    if enable_camera not in ["true", "1", "yes", "on"] or enable_rear_image_proc not in ["true", "1", "yes", "on"]:
        return []

    return [
        Node(
            package="image_proc",
            executable="debayer_node",
            namespace="camera_rear_1",
            name="image_proc",
            output="screen",
            remappings=[("image_raw", "image")],
        )
    ]


def _rear_image_proc_2_action(context):
    logger = get_logger("vehicle_platform_bringup")
    if not _package_available("image_proc"):
        logger.warning("Skipping rear camera_2 image_proc: package 'image_proc' is not available")
        return []

    enable_camera = LaunchConfiguration("enable_camera_rear_2").perform(context).lower()
    enable_rear_image_proc = LaunchConfiguration("enable_rear_image_proc").perform(context).lower()
    if enable_camera not in ["true", "1", "yes", "on"] or enable_rear_image_proc not in ["true", "1", "yes", "on"]:
        return []

    return [
        Node(
            package="image_proc",
            executable="debayer_node",
            namespace="camera_rear_2",
            name="image_proc",
            output="screen",
            remappings=[("image_raw", "image")],
        )
    ]


def _rear_image_proc_3_action(context):
    logger = get_logger("vehicle_platform_bringup")
    if not _package_available("image_proc"):
        logger.warning("Skipping rear camera_3 image_proc: package 'image_proc' is not available")
        return []

    enable_camera = LaunchConfiguration("enable_camera_rear_3").perform(context).lower()
    enable_rear_image_proc = LaunchConfiguration("enable_rear_image_proc").perform(context).lower()
    if enable_camera not in ["true", "1", "yes", "on"] or enable_rear_image_proc not in ["true", "1", "yes", "on"]:
        return []

    return [
        Node(
            package="image_proc",
            executable="debayer_node",
            namespace="camera_rear_3",
            name="image_proc",
            output="screen",
            remappings=[("image_raw", "image")],
        )
    ]


def _rviz_action(context):
    enable_rviz = LaunchConfiguration("enable_rviz").perform(context).lower()
    if enable_rviz not in ["true", "1", "yes", "on"]:
        return []

    rviz_config = LaunchConfiguration("rviz_config_file").perform(context).strip()
    rviz_args = []
    if rviz_config:
        rviz_args = ["-d", rviz_config]

    return [
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=rviz_args,
        )
    ]


def _package_available(package_name):
    try:
        get_package_share_directory(package_name)
        return True
    except PackageNotFoundError:
        return False


def _rear_camera_1_action(context):
    logger = get_logger("vehicle_platform_bringup")
    if not _package_available("arena_camera_node"):
        logger.warning("Skipping rear camera_1: package 'arena_camera_node' is not available")
        return []
    
    enable_camera = LaunchConfiguration("enable_camera_rear_1").perform(context).lower()
    if enable_camera not in ["true", "1", "yes", "on"]:
        return []
    
    return [
        Node(
            package="arena_camera_node",
            executable="start",
            namespace="camera_rear_1",
            name="arena_camera_node",
            output="log",
            arguments=["--ros-args", "--log-level", "warn"],
            parameters=[
                {
                    "topic": "/camera_rear_1/image",
                    "ip": LaunchConfiguration("rear_camera_1_ip"),
                    "host_iface_ip": LaunchConfiguration("rear_camera_1_host_iface_ip"),
                    "discovery_timeout_ms": LaunchConfiguration("camera_discovery_timeout_ms"),
                    "frame_rate": LaunchConfiguration("rear_camera_frame_rate_hz"),
                    "auto_exposure": LaunchConfiguration("rear_auto_exposure"),
                    "auto_exposure_min_us": LaunchConfiguration("rear_auto_exposure_min_us"),
                    "auto_exposure_max_us": LaunchConfiguration("rear_auto_exposure_max_us"),
                    "auto_exposure_target": LaunchConfiguration("rear_auto_exposure_target"),
                    "trigger_mode": LaunchConfiguration("rear_trigger_mode"),
                    "trigger_source": LaunchConfiguration("rear_trigger_source"),
                    "trigger_selector": LaunchConfiguration("rear_trigger_selector"),
                    "trigger_activation": LaunchConfiguration("rear_trigger_activation"),
                }
            ],
        )
    ]


def _rear_camera_2_action(context):
    logger = get_logger("vehicle_platform_bringup")
    if not _package_available("arena_camera_node"):
        logger.warning("Skipping rear camera_2: package 'arena_camera_node' is not available")
        return []
    
    enable_camera = LaunchConfiguration("enable_camera_rear_2").perform(context).lower()
    if enable_camera not in ["true", "1", "yes", "on"]:
        return []
    
    return [
        Node(
            package="arena_camera_node",
            executable="start",
            namespace="camera_rear_2",
            name="arena_camera_node",
            output="log",
            arguments=["--ros-args", "--log-level", "warn"],
            parameters=[
                {
                    "topic": "/camera_rear_2/image",
                    "ip": LaunchConfiguration("rear_camera_2_ip"),
                    "host_iface_ip": LaunchConfiguration("rear_camera_2_host_iface_ip"),
                    "discovery_timeout_ms": LaunchConfiguration("camera_discovery_timeout_ms"),
                    "frame_rate": LaunchConfiguration("rear_camera_frame_rate_hz"),
                    "auto_exposure": LaunchConfiguration("rear_auto_exposure"),
                    "auto_exposure_min_us": LaunchConfiguration("rear_auto_exposure_min_us"),
                    "auto_exposure_max_us": LaunchConfiguration("rear_auto_exposure_max_us"),
                    "auto_exposure_target": LaunchConfiguration("rear_auto_exposure_target"),
                    "trigger_mode": LaunchConfiguration("rear_trigger_mode"),
                    "trigger_source": LaunchConfiguration("rear_trigger_source"),
                    "trigger_selector": LaunchConfiguration("rear_trigger_selector"),
                    "trigger_activation": LaunchConfiguration("rear_trigger_activation"),
                }
            ],
        )
    ]


def _rear_camera_3_action(context):
    logger = get_logger("vehicle_platform_bringup")
    if not _package_available("arena_camera_node"):
        logger.warning("Skipping rear camera_3: package 'arena_camera_node' is not available")
        return []
    
    enable_camera = LaunchConfiguration("enable_camera_rear_3").perform(context).lower()
    if enable_camera not in ["true", "1", "yes", "on"]:
        return []
    
    return [
        Node(
            package="arena_camera_node",
            executable="start",
            namespace="camera_rear_3",
            name="arena_camera_node",
            output="log",
            arguments=["--ros-args", "--log-level", "warn"],
            parameters=[
                {
                    "topic": "/camera_rear_3/image",
                    "ip": LaunchConfiguration("rear_camera_3_ip"),
                    "host_iface_ip": LaunchConfiguration("rear_camera_3_host_iface_ip"),
                    "discovery_timeout_ms": LaunchConfiguration("camera_discovery_timeout_ms"),
                    "frame_rate": LaunchConfiguration("rear_camera_frame_rate_hz"),
                    "auto_exposure": LaunchConfiguration("rear_auto_exposure"),
                    "auto_exposure_min_us": LaunchConfiguration("rear_auto_exposure_min_us"),
                    "auto_exposure_max_us": LaunchConfiguration("rear_auto_exposure_max_us"),
                    "auto_exposure_target": LaunchConfiguration("rear_auto_exposure_target"),
                    "trigger_mode": LaunchConfiguration("rear_trigger_mode"),
                    "trigger_source": LaunchConfiguration("rear_trigger_source"),
                    "trigger_selector": LaunchConfiguration("rear_trigger_selector"),
                    "trigger_activation": LaunchConfiguration("rear_trigger_activation"),
                }
            ],
        )
    ]


def generate_launch_description():
    vehicle_platform_share = get_package_share_directory("vehicle_platform")
    config_dir = os.path.join(vehicle_platform_share, "config")
    delphi_share = get_package_share_directory("delphi_esr_driver")
    velodyne_driver_share = get_package_share_directory("velodyne_driver")
    novatel_share = get_package_share_directory("novatel_oem7_driver")
    v2x_share = get_package_share_directory("v2x_ros_driver")
    dbw_share = get_package_share_directory("raptor_dbw_can")
    rviz_default_config = os.path.join(vehicle_platform_share, "sample_file.rviz")
    camera_fl_info_default = "file://" + os.path.join(config_dir, "camera_fl.intrinsics.yaml")
    rear_camera_rc_info_default = "file://" + os.path.join(
        config_dir, "camera_rc_arena_camera_node.intrinsics.yaml"
    )
    rear_camera_tl_info_default = "file://" + os.path.join(
        config_dir, "camera_tl_arena_camera_node.intrinsics.yaml"
    )
    rear_camera_tr_info_default = "file://" + os.path.join(
        config_dir, "camera_tr_arena_camera_node.intrinsics.yaml"
    )
    front_camera_params_default = os.path.join(config_dir, "avt_mako_15hz.yaml")

    lidar_tl_driver_params = os.path.join(
        velodyne_driver_share, "config", "Lidar_tl_VLP16-velodyne_driver_node-params.yaml"
    )
    lidar_tr_driver_params = os.path.join(
        velodyne_driver_share, "config", "Lidar_tr_VLP16-velodyne_driver_node-params.yaml"
    )
    lidar_tc_driver_params = os.path.join(
        velodyne_driver_share, "config", "VLP32C-velodyne_driver_node-params.yaml"
    )

    novatel_launch = os.path.join(novatel_share, "launch", "oem7_net.launch.py")
    v2x_launch = os.path.join(v2x_share, "launch", "v2x_ros_driver.launch.py")
    v2x_override_file = "/home/avalocal/ros_drivers/src/v2x-ros-driver/v2x_ros_driver/GlobalParamsOverride.c2p.yaml"
    dbw_launch = os.path.join(dbw_share, "launch", "raptor_dbw_can_launch.py")
    delphi_launch = os.path.join(delphi_share, "launch", "delphi_esr_bringup.launch.py")
    extrinsics_launch = os.path.join(vehicle_platform_share, "launch", "extrinsics.launch.py")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_zenohd",
                default_value="true",
                description="Start rmw_zenohd from this launch",
            ),
            DeclareLaunchArgument(
                "camera_discovery_timeout_ms",
                default_value="10000",
                description="Arena camera discovery timeout in milliseconds",
            ),
            DeclareLaunchArgument(
                "front_camera_fl_ip",
                default_value="192.168.20.10",
                description="Front-left camera IP for avt_vimba driver",
            ),
            DeclareLaunchArgument(
                "front_camera_fl_info_url",
                default_value=camera_fl_info_default,
                description="Calibration file URL for front-left camera (camera_info_url)",
            ),
            DeclareLaunchArgument(
                "front_camera_fr_ip",
                default_value="192.168.10.10",
                description="Front-right camera IP for avt_vimba driver",
            ),
            DeclareLaunchArgument(
                "front_camera_params_file",
                default_value=front_camera_params_default,
                description="Front camera AVT parameter file (use this to cap front camera frame rate)",
            ),
            DeclareLaunchArgument(
                "rear_camera_rc_info_url",
                default_value=rear_camera_rc_info_default,
                description="Calibration file URL for rear-center camera",
            ),
            DeclareLaunchArgument(
                "rear_camera_tl_info_url",
                default_value=rear_camera_tl_info_default,
                description="Calibration file URL for rear top-left camera",
            ),
            DeclareLaunchArgument(
                "rear_camera_tr_info_url",
                default_value=rear_camera_tr_info_default,
                description="Calibration file URL for rear top-right camera",
            ),
            DeclareLaunchArgument(
                "enable_camera_fl",
                default_value="true",
                description="Enable the front-left avt_vimba camera",
            ),
            DeclareLaunchArgument(
                "enable_camera_fr",
                default_value="false",
                description="Enable the front-right avt_vimba camera",
            ),
            DeclareLaunchArgument(
                "enable_front_image_proc",
                default_value="true",
                description="Enable avt_vimba image_proc debayer pipeline for front cameras",
            ),
            DeclareLaunchArgument(
                "rear_camera_1_ip",
                default_value="192.168.50.10",
                description="Rear camera 1 IP for arena camera driver",
            ),
            DeclareLaunchArgument(
                "rear_camera_2_ip",
                default_value="192.168.40.10",
                description="Rear camera 2 IP for arena camera driver",
            ),
            DeclareLaunchArgument(
                "rear_camera_3_ip",
                default_value="192.168.30.10",
                description="Rear camera 3 IP for arena camera driver",
            ),
            DeclareLaunchArgument(
                "rear_camera_1_host_iface_ip",
                default_value="192.168.50.100",
                description="Host NIC IP to use for rear camera 1 discovery",
            ),
            DeclareLaunchArgument(
                "rear_camera_2_host_iface_ip",
                default_value="192.168.40.100",
                description="Host NIC IP to use for rear camera 2 discovery",
            ),
            DeclareLaunchArgument(
                "rear_camera_3_host_iface_ip",
                default_value="192.168.30.100",
                description="Host NIC IP to use for rear camera 3 discovery",
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
                default_value="100.0",
                description="Rear camera minimum exposure limit in microseconds",
            ),
            DeclareLaunchArgument(
                "rear_auto_exposure_max_us",
                default_value="60000.0",
                description="Rear camera maximum exposure limit in microseconds",
            ),
            DeclareLaunchArgument(
                "rear_auto_exposure_target",
                default_value="50.0",
                description="Rear camera auto exposure target brightness",
            ),
            DeclareLaunchArgument(
                "rear_trigger_mode",
                default_value="false",
                description="Enable rear camera trigger mode (default false for stable free-run startup)",
            ),
            DeclareLaunchArgument(
                "rear_trigger_source",
                default_value="Freerun",
                description="Rear camera trigger source (default Freerun)",
            ),
            DeclareLaunchArgument(
                "rear_trigger_selector",
                default_value="FrameStart",
                description="Rear camera trigger selector (default FrameStart)",
            ),
            DeclareLaunchArgument(
                "rear_trigger_activation",
                default_value="FixedRate",
                description="Rear camera trigger activation (default FixedRate)",
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
                "enable_rear_image_proc",
                default_value="true",
                description="Enable image_proc debayer for rear camera image topics",
            ),
            DeclareLaunchArgument(
                "enable_rviz",
                default_value="true",
                description="Enable RViz startup",
            ),
            DeclareLaunchArgument(
                "enable_map_odom_tf",
                default_value="true",
                description="Enable dynamic map->odom TF from Novatel odom + GPS",
            ),
            DeclareLaunchArgument(
                "map_frame",
                default_value="map",
                description="Global frame id for dynamic map->odom TF",
            ),
            DeclareLaunchArgument(
                "odom_frame",
                default_value="odom",
                description="Odometry frame id for dynamic map->odom TF",
            ),
            DeclareLaunchArgument(
                "odom_child_frame",
                default_value="lidar_tc",
                description="Child frame attached to odom using Novatel odometry pose (connects sensor tree)",
            ),
            DeclareLaunchArgument(
                "use_odom_msg_child_frame",
                default_value="false",
                description="If true, use nav_msgs/Odometry child_frame_id instead of odom_child_frame",
            ),
            DeclareLaunchArgument(
                "novatel_odom_topic",
                default_value="/novatel/oem7/odom",
                description="Novatel odometry topic used as odom source",
            ),
            DeclareLaunchArgument(
                "novatel_gps_topic",
                default_value="/novatel/oem7/fix",
                description="Novatel GPS topic (NavSatFix) used as map anchor source",
            ),
            DeclareLaunchArgument(
                "map_odom_tf_rate_hz",
                default_value="20.0",
                description="Publish rate for dynamic map->odom TF",
            ),
            DeclareLaunchArgument(
                "map_use_fixed_reference",
                default_value="false",
                description="If true, keep map frame anchored at fixed global LLA origin",
            ),
            DeclareLaunchArgument(
                "map_fixed_reference_lat_deg",
                default_value="30.63789534",
                description="Fixed global map origin latitude (deg)",
            ),
            DeclareLaunchArgument(
                "map_fixed_reference_lon_deg",
                default_value="-96.47777581",
                description="Fixed global map origin longitude (deg)",
            ),
            DeclareLaunchArgument(
                "map_fixed_reference_alt_m",
                default_value="55.09027857",
                description="Fixed global map origin altitude (m)",
            ),
            DeclareLaunchArgument(
                "map_anchor_mode",
                default_value="map_then_lock",
                description="Map anchor mode for TF node: gps_fallback | map_then_lock | map_only",
            ),
            DeclareLaunchArgument(
                "map_anchor_topic",
                default_value="/message/incoming_map",
                description="Decoded MAP topic used when map_anchor_mode requires MAP anchoring",
            ),
            DeclareLaunchArgument(
                "inbound_map_anchor_topic",
                default_value="/comms/inbound_binary_msg",
                description="Raw inbound V2X topic used when map_anchor_mode requires MAP anchoring from inbound stream",
            ),
            DeclareLaunchArgument(
                "map_odom_use_gps_altitude",
                default_value="false",
                description="If true, include GPS altitude delta in map->odom translation",
            ),
            DeclareLaunchArgument(
                "enable_delphi_esr",
                default_value="true",
                description="Enable Delphi ESR 2.5 radar bringup",
            ),
            DeclareLaunchArgument(
                "delphi_namespace",
                default_value="delphi_esr_interface",
                description="Namespace for Delphi ESR nodes",
            ),
            DeclareLaunchArgument(
                "delphi_params",
                default_value=os.path.join(delphi_share, "config", "delphi_esr_params.yaml"),
                description="Path to Delphi ESR driver parameter file",
            ),
            DeclareLaunchArgument(
                "delphi_hardware_id",
                default_value="94492",
                description="Kvaser hardware serial for Delphi ESR radar",
            ),
            DeclareLaunchArgument(
                "delphi_circuit_id",
                default_value="0",
                description="Kvaser circuit id for Delphi ESR radar",
            ),
            DeclareLaunchArgument(
                "delphi_bit_rate",
                default_value="500000",
                description="CAN bit rate for Delphi ESR radar",
            ),
            DeclareLaunchArgument(
                "delphi_enable_echo",
                default_value="false",
                description="Enable Kvaser echo in Delphi ESR bringup",
            ),
            DeclareLaunchArgument(
                "delphi_tx_enable",
                default_value="false",
                description="Enable Delphi ESR command TX (0x4F0/0x4F1)",
            ),
            DeclareLaunchArgument(
                "delphi_tx_rate_hz",
                default_value="20.0",
                description="Delphi ESR TX command rate in Hz",
            ),
            DeclareLaunchArgument(
                "rviz_config_file",
                default_value=rviz_default_config,
                description="RViz config file path",
            ),
            DeclareLaunchArgument(
                "enable_v2x_driver_lifecycle",
                default_value="true",
                description="Enable manual lifecycle configure/activate commands for v2x driver",
            ),
            DeclareLaunchArgument(
                "enable_map_spat_visualizer",
                default_value="false",
                description="Enable pre-decoded MAP/SPAT visualizer (requires external decoded /message/incoming_map and /message/incoming_spat publishers)",
            ),
            DeclareLaunchArgument(
                "enable_inbound_binary_visualizer",
                default_value="true",
                description="Enable decoder-backed visualizer from /comms/inbound_binary_msg",
            ),
            DeclareLaunchArgument(
                "v2x_inbound_marker_topic",
                default_value="/v2x/map_spat_markers",
                description="Output marker topic for inbound decoder-backed visualizer",
            ),
            DeclareLaunchArgument(
                "v2x_visualization_frame",
                default_value="map",
                description="TF frame used by V2X MAP/SPAT/BSM/PSM/TIM marker visualization",
            ),
            DeclareLaunchArgument(
                "v2x_configuration_delay",
                default_value="8.0",
                description="Delay in seconds before running v2x lifecycle configure",
            ),
            DeclareLaunchArgument(
                "enable_safety_alert_bridge",
                default_value="false",
                description="Enable SDK-backed V2X safety alert bridge publishing /v2x/safety_alerts",
            ),
            DeclareLaunchArgument(
                "enable_native_safety_alert_bridge",
                default_value="true",
                description="Enable native C++ Commsignia app-notif safety bridge (requires SDK-enabled build)",
            ),
            DeclareLaunchArgument(
                "v2x_safety_alert_abbrev_overlay_topic",
                default_value="/v2x/safety_alert_overlay_text",
                description="RViz 2D overlay topic for active safety alert abbreviation text",
            ),
            DeclareLaunchArgument(
                "v2x_safety_alert_enable_abbrev_overlay",
                default_value="true",
                description="Enable RViz 2D overlay text publishing for safety alert abbreviations",
            ),
            DeclareLaunchArgument(
                "v2x_safety_bridge_obu_host",
                default_value="192.168.0.54",
                description="OBU host IP for SDK safety bridge RPC",
            ),
            DeclareLaunchArgument(
                "v2x_safety_bridge_obu_port",
                default_value="43985",
                description="OBU app-notif UDP port for native C++ bridge",
            ),
            DeclareLaunchArgument(
                "v2x_safety_bridge_local_port",
                default_value="0",
                description="Local UDP port for native C++ bridge client",
            ),
            DeclareLaunchArgument(
                "v2x_safety_bridge_notif_filter_csv",
                default_value="",
                description="Optional comma-separated native notif filters (e.g., FCW,IMA,WWE,WWR)",
            ),
            DeclareLaunchArgument(
                "v2x_safety_bridge_reconnect_delay",
                default_value="2.0",
                description="Reconnect delay in seconds for SDK safety bridge",
            ),
            DeclareLaunchArgument(
                "v2x_safety_bridge_subscription_key",
                default_value="0",
                description="SDK fac_subscribe key (0 subscribes to all facility message types)",
            ),
            DeclareLaunchArgument(
                "v2x_safety_bridge_derive_cff_only",
                default_value="true",
                description="If true, publish only alerts derived from CFF/collision indicators",
            ),
            DeclareLaunchArgument(
                "v2x_safety_bridge_publish_raw_passthrough",
                default_value="true",
                description="If true, include raw SDK event payload in published alerts for debugging",
            ),
            DeclareLaunchArgument(
                "v2x_safety_bridge_dedupe_window_sec",
                default_value="0.75",
                description="Minimum seconds between identical alert signatures",
            ),
            DeclareLaunchArgument(
                "v2x_safety_bridge_critical_ttc_sec",
                default_value="2.0",
                description="TTC threshold at or below which alert severity is critical",
            ),
            DeclareLaunchArgument(
                "v2x_safety_bridge_warning_ttc_sec",
                default_value="4.0",
                description="TTC threshold at or below which alert severity is warning",
            ),
            DeclareLaunchArgument(
                "v2x_global_params_override_file",
                default_value=v2x_override_file,
                description="Path to v2x global override file (set listening_port to match OBU mode)",
            ),
            SetEnvironmentVariable(
                name="LD_LIBRARY_PATH",
                value=[
                    "/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu:",
                    EnvironmentVariable("LD_LIBRARY_PATH", default_value=""),
                ],
            ),
            SetEnvironmentVariable(name="RMW_IMPLEMENTATION", value="rmw_zenoh_cpp"),
            ExecuteProcess(
                cmd=["ros2", "run", "rmw_zenoh_cpp", "rmw_zenohd"],
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_zenohd")),
            ),
            _velodyne_group(
                namespace="lidar_tl",
                frame_id="lidar_tl",
                driver_params_file=lidar_tl_driver_params,
                transform_cfg_name="VLP16-velodyne_transform_node-params.yaml",
                calibration_name="VLP16db.yaml",
            ),
            _velodyne_group(
                namespace="lidar_tr",
                frame_id="lidar_tr",
                driver_params_file=lidar_tr_driver_params,
                transform_cfg_name="VLP16-velodyne_transform_node-params.yaml",
                calibration_name="VLP16db.yaml",
            ),
            _velodyne_group(
                namespace="lidar_tc",
                frame_id="lidar_tc",
                driver_params_file=lidar_tc_driver_params,
                transform_cfg_name="VLP32C-velodyne_transform_node-params.yaml",
                calibration_name="VeloView-VLP-32C.yaml",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(novatel_launch),
                launch_arguments={
                    "oem7_ip_addr": "192.168.100.150",
                    "oem7_port": "3004",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(v2x_launch),
                launch_arguments={
                    "enable_v2x_driver_lifecycle": LaunchConfiguration("enable_v2x_driver_lifecycle"),
                    "configuration_delay": LaunchConfiguration("v2x_configuration_delay"),
                    "enable_map_spat_visualizer": LaunchConfiguration("enable_map_spat_visualizer"),
                    "enable_inbound_binary_visualizer": LaunchConfiguration("enable_inbound_binary_visualizer"),
                    "enable_safety_alert_bridge": LaunchConfiguration("enable_safety_alert_bridge"),
                    "enable_native_safety_alert_bridge": LaunchConfiguration("enable_native_safety_alert_bridge"),
                    "safety_alert_abbrev_overlay_topic": LaunchConfiguration("v2x_safety_alert_abbrev_overlay_topic"),
                    "safety_alert_enable_abbrev_overlay": LaunchConfiguration("v2x_safety_alert_enable_abbrev_overlay"),
                    "safety_bridge_obu_host": LaunchConfiguration("v2x_safety_bridge_obu_host"),
                    "safety_bridge_obu_port": LaunchConfiguration("v2x_safety_bridge_obu_port"),
                    "safety_bridge_local_port": LaunchConfiguration("v2x_safety_bridge_local_port"),
                    "safety_bridge_notif_filter_csv": LaunchConfiguration("v2x_safety_bridge_notif_filter_csv"),
                    "safety_bridge_reconnect_delay": LaunchConfiguration("v2x_safety_bridge_reconnect_delay"),
                    "safety_bridge_subscription_key": LaunchConfiguration("v2x_safety_bridge_subscription_key"),
                    "safety_bridge_derive_cff_only": LaunchConfiguration("v2x_safety_bridge_derive_cff_only"),
                    "safety_bridge_publish_raw_passthrough": LaunchConfiguration("v2x_safety_bridge_publish_raw_passthrough"),
                    "safety_bridge_dedupe_window_sec": LaunchConfiguration("v2x_safety_bridge_dedupe_window_sec"),
                    "safety_bridge_critical_ttc_sec": LaunchConfiguration("v2x_safety_bridge_critical_ttc_sec"),
                    "safety_bridge_warning_ttc_sec": LaunchConfiguration("v2x_safety_bridge_warning_ttc_sec"),
                    "inbound_marker_topic": LaunchConfiguration("v2x_inbound_marker_topic"),
                    "visualization_frame_id": LaunchConfiguration("v2x_visualization_frame"),
                    "use_fixed_global_anchor": LaunchConfiguration("map_use_fixed_reference"),
                    "fixed_anchor_lat_deg": LaunchConfiguration("map_fixed_reference_lat_deg"),
                    "fixed_anchor_lon_deg": LaunchConfiguration("map_fixed_reference_lon_deg"),
                    "global_params_override_file": LaunchConfiguration("v2x_global_params_override_file"),
                }.items(),
            ),
            OpaqueFunction(function=_front_camera_fl_action),
            OpaqueFunction(function=_front_camera_fr_action),
            OpaqueFunction(function=_front_image_proc_fl_action),
            OpaqueFunction(function=_front_image_proc_fr_action),
            OpaqueFunction(function=_rear_camera_1_action),
            OpaqueFunction(function=_rear_camera_2_action),
            OpaqueFunction(function=_rear_camera_3_action),
            OpaqueFunction(function=_rear_image_proc_1_action),
            OpaqueFunction(function=_rear_image_proc_2_action),
            OpaqueFunction(function=_rear_image_proc_3_action),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(extrinsics_launch)),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(dbw_launch)),
            Node(
                package="vehicle_platform",
                executable="map_odom_tf_from_gps.py",
                name="map_odom_tf_from_gps",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_map_odom_tf")),
                parameters=[
                    {
                        "map_frame": LaunchConfiguration("map_frame"),
                        "odom_frame": LaunchConfiguration("odom_frame"),
                        "odom_child_frame": LaunchConfiguration("odom_child_frame"),
                        "use_odom_msg_child_frame": LaunchConfiguration("use_odom_msg_child_frame"),
                        "odom_topic": LaunchConfiguration("novatel_odom_topic"),
                        "gps_topic": LaunchConfiguration("novatel_gps_topic"),
                        "publish_rate_hz": LaunchConfiguration("map_odom_tf_rate_hz"),
                        "use_navsat_altitude": LaunchConfiguration("map_odom_use_gps_altitude"),
                        "use_fixed_reference": LaunchConfiguration("map_use_fixed_reference"),
                        "fixed_reference_lat_deg": LaunchConfiguration("map_fixed_reference_lat_deg"),
                        "fixed_reference_lon_deg": LaunchConfiguration("map_fixed_reference_lon_deg"),
                        "fixed_reference_alt_m": LaunchConfiguration("map_fixed_reference_alt_m"),
                        "anchor_mode": LaunchConfiguration("map_anchor_mode"),
                        "map_anchor_topic": LaunchConfiguration("map_anchor_topic"),
                        "inbound_map_anchor_topic": LaunchConfiguration("inbound_map_anchor_topic"),
                        "allow_anchor_relock": False,
                        "reanchor_distance_m": 150.0,
                    }
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(delphi_launch),
                condition=IfCondition(LaunchConfiguration("enable_delphi_esr")),
                launch_arguments={
                    "namespace": LaunchConfiguration("delphi_namespace"),
                    "params": LaunchConfiguration("delphi_params"),
                    "hardware_id": LaunchConfiguration("delphi_hardware_id"),
                    "circuit_id": LaunchConfiguration("delphi_circuit_id"),
                    "bit_rate": LaunchConfiguration("delphi_bit_rate"),
                    "enable_echo": LaunchConfiguration("delphi_enable_echo"),
                    "tx_enable": LaunchConfiguration("delphi_tx_enable"),
                    "tx_rate_hz": LaunchConfiguration("delphi_tx_rate_hz"),
                    "open_rviz": "false",
                }.items(),
            ),
            OpaqueFunction(function=_rviz_action),
        ]
    )
