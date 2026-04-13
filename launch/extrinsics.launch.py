import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _extrinsics_nodes(context):
    extrinsics_file = LaunchConfiguration("extrinsics_file").perform(context)
    with open(extrinsics_file, "r", encoding="utf-8") as f:
        extrinsics = yaml.safe_load(f)

    nodes = []
    for key, entry in extrinsics.items():
        if not isinstance(entry, dict):
            continue
        parent = entry["parent"]
        child = entry["child"]
        x, y, z, qx, qy, qz, qw = entry["value"]
        nodes.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name=f"static_tf_{key}",
                arguments=[
                    str(x), str(y), str(z),
                    str(qx), str(qy), str(qz), str(qw),
                    parent, child,
                ],
                output="screen",
            )
        )
    return nodes


def generate_launch_description():
    vehicle_platform_share = get_package_share_directory("vehicle_platform")
    default_extrinsics = os.path.join(vehicle_platform_share, "config", "extrinsics.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "extrinsics_file",
                default_value=default_extrinsics,
                description="Path to extrinsics YAML file",
            ),
            OpaqueFunction(function=_extrinsics_nodes),
        ]
    )
