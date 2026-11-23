import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _package_available(package_name):
    try:
        get_package_share_directory(package_name)
        return True
    except PackageNotFoundError:
        return False


def _front_camera_action(context, camera_name):
    logger = get_logger("front_cameras_only")
    if not _package_available("avt_vimba_camera"):
        logger.warning("Skipping %s: package 'avt_vimba_camera' is not available" % camera_name)
        return []

    enabled = LaunchConfiguration("enable_%s" % camera_name).perform(context).lower()
    if enabled not in ["true", "1", "yes", "on"]:
        return []

    camera_suffix = camera_name.replace("camera_", "")
    avt_share = get_package_share_directory("avt_vimba_camera")
    mako_launch = os.path.join(avt_share, "launch", "Mako_G-319.launch.xml")

    launch_args = {
        "name": camera_name,
        "frame_id": camera_name,
        "ip": LaunchConfiguration("front_camera_%s_ip" % camera_suffix).perform(context),
        "params_file": LaunchConfiguration("front_camera_params_file").perform(context),
        "use_measurement_time": "false",
        "image_proc": "false",
    }

    if camera_name == "camera_fl":
        launch_args["camera_info_url"] = LaunchConfiguration("front_camera_fl_info_url").perform(context)

    return [
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(mako_launch),
            launch_arguments=launch_args.items(),
        )
    ]


def _front_image_proc_action(context, camera_name):
    logger = get_logger("front_cameras_only")
    if not _package_available("image_proc"):
        logger.warning("Skipping %s image_proc: package 'image_proc' is not available" % camera_name)
        return []

    enabled = LaunchConfiguration("enable_%s" % camera_name).perform(context).lower()
    enable_front_image_proc = LaunchConfiguration("enable_front_image_proc").perform(context).lower()
    if enabled not in ["true", "1", "yes", "on"] or enable_front_image_proc not in [
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
            namespace=camera_name,
            name="image_proc",
            output="screen",
            # 0 = bilinear; the default (3 = VNG) cannot keep up at 15 Hz on 3 MP frames
            parameters=[{"debayer": 0}],
            remappings=[("image_raw", "image")],
        )
    ]


def generate_launch_description():
    vehicle_platform_share = get_package_share_directory("vehicle_platform")
    config_dir = os.path.join(vehicle_platform_share, "config")
    camera_fl_info_default = "file://" + os.path.join(config_dir, "camera_fl.intrinsics.yaml")
    front_camera_params_default = os.path.join(config_dir, "avt_mako_15hz.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_camera_fl",
                default_value="true",
                description="Enable the front-left AVT camera",
            ),
            DeclareLaunchArgument(
                "enable_camera_fr",
                default_value="true",
                description="Enable the front-right AVT camera",
            ),
            DeclareLaunchArgument(
                "enable_front_image_proc",
                default_value="true",
                description="Enable AVT image_proc debayer for the front cameras",
            ),
            DeclareLaunchArgument(
                "front_camera_fl_ip",
                default_value="192.168.20.10",
                description="Front-left camera IP for avt_vimba driver",
            ),
            DeclareLaunchArgument(
                "front_camera_fr_ip",
                default_value="192.168.10.10",
                description="Front-right camera IP for avt_vimba driver",
            ),
            DeclareLaunchArgument(
                "front_camera_fl_info_url",
                default_value=camera_fl_info_default,
                description="Calibration for the front-left camera",
            ),
            DeclareLaunchArgument(
                "front_camera_params_file",
                default_value=front_camera_params_default,
                description="AVT parameter file for front camera timing and rate limits",
            ),
            OpaqueFunction(function=lambda context: _front_camera_action(context, "camera_fl")),
            OpaqueFunction(function=lambda context: _front_camera_action(context, "camera_fr")),
            OpaqueFunction(function=lambda context: _front_image_proc_action(context, "camera_fl")),
            OpaqueFunction(function=lambda context: _front_image_proc_action(context, "camera_fr")),
        ]
    )
