"""Vehicle-aware entry point for the full sensor bringup.

Each vehicle keeps its own bringup in launch/vehicles/<vehicle>.launch.py
and its own calibration/sensor configs:

    blue_jeep   launch/vehicles/blue_jeep.launch.py   config/
    white_jeep  launch/vehicles/white_jeep.launch.py  config/white_jeep_specific/

Pick the vehicle with `vehicle:=<name>`, or set VEHICLE_PLATFORM_VEHICLE once
per machine. Without either, the blue jeep bringup runs. Every other launch
argument is passed straight through to the selected bringup.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration

VEHICLES = ("blue_jeep", "white_jeep")


def _include_vehicle_bringup(context):
    vehicle = LaunchConfiguration("vehicle").perform(context).strip()
    if vehicle not in VEHICLES:
        raise RuntimeError(
            "Unknown vehicle '{}'. Valid values: {}".format(vehicle, ", ".join(VEHICLES))
        )
    bringup = os.path.join(
        get_package_share_directory("vehicle_platform"),
        "launch",
        "vehicles",
        vehicle + ".launch.py",
    )
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(bringup))]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "vehicle",
                default_value=EnvironmentVariable(
                    "VEHICLE_PLATFORM_VEHICLE", default_value="blue_jeep"
                ),
                description="Vehicle to bring up: " + " | ".join(VEHICLES)
                + " (default: $VEHICLE_PLATFORM_VEHICLE, else blue_jeep)",
            ),
            OpaqueFunction(function=_include_vehicle_bringup),
        ]
    )
