import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


VEHICLE_EXTRINSICS = {
    "blue_jeep": ("config", "extrinsics.yaml"),
    "white_jeep": ("config", "white_jeep_specific", "extrinsics.yaml"),
}


def _extrinsics_nodes(context):
    extrinsics_file = LaunchConfiguration("extrinsics_file").perform(context).strip()
    if not extrinsics_file:
        vehicle = LaunchConfiguration("vehicle").perform(context).strip()
        if vehicle not in VEHICLE_EXTRINSICS:
            raise RuntimeError(
                "Unknown vehicle '{}'. Valid values: {}".format(
                    vehicle, ", ".join(VEHICLE_EXTRINSICS)
                )
            )
        extrinsics_file = os.path.join(
            get_package_share_directory("vehicle_platform"), *VEHICLE_EXTRINSICS[vehicle]
        )
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
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "vehicle",
                default_value=EnvironmentVariable(
                    "VEHICLE_PLATFORM_VEHICLE", default_value="blue_jeep"
                ),
                description="Vehicle whose extrinsics to publish: "
                + " | ".join(VEHICLE_EXTRINSICS),
            ),
            DeclareLaunchArgument(
                "extrinsics_file",
                default_value="",
                description="Path to extrinsics YAML file (empty: use the vehicle's file)",
            ),
            OpaqueFunction(function=_extrinsics_nodes),
        ]
    )
