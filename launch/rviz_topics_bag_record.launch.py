#!/usr/bin/env python3

import os
import re

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _extract_topics_from_rviz_config(rviz_config_path):
    topic_values = set()
    topic_pattern = re.compile(r"^\s*[A-Za-z_ ]*Topic\s*:\s*(.+?)\s*$")

    with open(rviz_config_path, "r", encoding="utf-8") as rviz_file:
        for line in rviz_file:
            match = topic_pattern.match(line)
            if not match:
                continue

            value = match.group(1).strip().strip("\"'")
            if not value or value.startswith("${"):
                continue
            if not value.startswith("/"):
                continue
            topic_values.add(value)

    return sorted(topic_values)


def _build_bag_record_action(context):
    rviz_config = LaunchConfiguration("rviz_config").perform(context)
    bag_output = LaunchConfiguration("bag_output").perform(context)

    if not rviz_config:
        raise RuntimeError("rviz_config launch argument is required")
    if not os.path.isfile(rviz_config):
        raise RuntimeError("RViz config file not found: {}".format(rviz_config))

    topics = _extract_topics_from_rviz_config(rviz_config)
    if not topics:
        raise RuntimeError("No topics found in RViz config: {}".format(rviz_config))

    bag_cmd = ["ros2", "bag", "record", "-o", bag_output] + topics

    return [
        LogInfo(msg="Recording {} topic(s) from {}".format(len(topics), rviz_config)),
        ExecuteProcess(cmd=bag_cmd, output="screen"),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz_config",
                default_value="",
                description="Absolute path to RViz config file to read topics from",
            ),
            DeclareLaunchArgument(
                "bag_output",
                default_value="rviz_topics_bag",
                description="Output rosbag name/path",
            ),
            OpaqueFunction(function=_build_bag_record_action),
        ]
    )