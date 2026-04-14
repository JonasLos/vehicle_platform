import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    vehicle_platform_share = get_package_share_directory("vehicle_platform")
    base_launch = os.path.join(vehicle_platform_share, "launch", "system_bringup.launch.py")

    # Test-only alternate TF path:
    # - keep existing bringup unchanged
    # - disable default map_odom_tf_from_gps inside base launch
    # - start a second map_odom_tf_from_gps instance with fixed GPS map origin if enabled
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_alt_map_odom_tf",
                default_value="true",
                description="Enable alternate test map->odom TF publisher",
            ),
            DeclareLaunchArgument(
                "alt_map_frame",
                default_value="map",
                description="Global frame for alternate map->odom TF",
            ),
            DeclareLaunchArgument(
                "alt_odom_frame",
                default_value="odom",
                description="Odometry frame for alternate map->odom TF",
            ),
            DeclareLaunchArgument(
                "alt_odom_child_frame",
                default_value="lidar_tc",
                description="Child frame attached to odom by alternate TF node",
            ),
            DeclareLaunchArgument(
                "alt_use_odom_msg_child_frame",
                default_value="false",
                description="Use child_frame_id from Odometry message when true",
            ),
            DeclareLaunchArgument(
                "alt_novatel_odom_topic",
                default_value="/novatel/oem7/odom",
                description="Odometry topic for alternate map->odom TF",
            ),
            DeclareLaunchArgument(
                "alt_novatel_gps_topic",
                default_value="/novatel/oem7/fix",
                description="GPS topic for alternate map->odom TF",
            ),
            DeclareLaunchArgument(
                "alt_map_odom_tf_rate_hz",
                default_value="20.0",
                description="Publish rate for alternate map->odom TF",
            ),
            DeclareLaunchArgument(
                "alt_map_odom_use_gps_altitude",
                default_value="false",
                description="If true, include GPS altitude delta in alternate map->odom translation",
            ),
            DeclareLaunchArgument(
                "alt_use_fixed_reference",
                default_value="false",
                description="Use fixed GPS reference for alternate TF (recommended for MAP/SPAT alignment test)",
            ),
            DeclareLaunchArgument(
                "alt_anchor_mode",
                default_value="map_only",
                description="Anchor mode for alternate TF: map_only | map_then_lock | gps_fallback",
            ),
            DeclareLaunchArgument(
                "alt_map_anchor_topic",
                default_value="/message/incoming_map",
                description="Decoded MAP topic used to set fixed map anchor in alternate TF",
            ),
            DeclareLaunchArgument(
                "alt_inbound_map_anchor_topic",
                default_value="/comms/inbound_binary_msg",
                description="Raw inbound V2X topic used to derive MAP anchor in alternate TF",
            ),
            DeclareLaunchArgument(
                "alt_allow_anchor_relock",
                default_value="false",
                description="Allow alternate TF anchor to move when MAP indicates a far-away location",
            ),
            DeclareLaunchArgument(
                "alt_reanchor_distance_m",
                default_value="150.0",
                description="Distance threshold for MAP anchor shift when relock is enabled",
            ),
            DeclareLaunchArgument(
                "alt_fixed_reference_lat_deg",
                default_value="0.0",
                description="Fixed map origin latitude for alternate TF",
            ),
            DeclareLaunchArgument(
                "alt_fixed_reference_lon_deg",
                default_value="0.0",
                description="Fixed map origin longitude for alternate TF",
            ),
            DeclareLaunchArgument(
                "alt_fixed_reference_alt_m",
                default_value="0.0",
                description="Fixed map origin altitude for alternate TF",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(base_launch),
                launch_arguments={
                    "enable_map_odom_tf": "false",
                }.items(),
            ),
            Node(
                package="vehicle_platform",
                executable="map_odom_tf_from_gps.py",
                name="map_odom_tf_from_gps_alt",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_alt_map_odom_tf")),
                parameters=[
                    {
                        "map_frame": LaunchConfiguration("alt_map_frame"),
                        "odom_frame": LaunchConfiguration("alt_odom_frame"),
                        "odom_child_frame": LaunchConfiguration("alt_odom_child_frame"),
                        "use_odom_msg_child_frame": LaunchConfiguration("alt_use_odom_msg_child_frame"),
                        "odom_topic": LaunchConfiguration("alt_novatel_odom_topic"),
                        "gps_topic": LaunchConfiguration("alt_novatel_gps_topic"),
                        "publish_rate_hz": LaunchConfiguration("alt_map_odom_tf_rate_hz"),
                        "use_navsat_altitude": LaunchConfiguration("alt_map_odom_use_gps_altitude"),
                        "anchor_mode": LaunchConfiguration("alt_anchor_mode"),
                        "map_anchor_topic": LaunchConfiguration("alt_map_anchor_topic"),
                        "inbound_map_anchor_topic": LaunchConfiguration("alt_inbound_map_anchor_topic"),
                        "allow_anchor_relock": LaunchConfiguration("alt_allow_anchor_relock"),
                        "reanchor_distance_m": LaunchConfiguration("alt_reanchor_distance_m"),
                        "use_fixed_reference": LaunchConfiguration("alt_use_fixed_reference"),
                        "fixed_reference_lat_deg": LaunchConfiguration("alt_fixed_reference_lat_deg"),
                        "fixed_reference_lon_deg": LaunchConfiguration("alt_fixed_reference_lon_deg"),
                        "fixed_reference_alt_m": LaunchConfiguration("alt_fixed_reference_alt_m"),
                    }
                ],
            ),
        ]
    )
