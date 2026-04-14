#!/usr/bin/env python3

import argparse
import os
import re
import shlex
import subprocess
import sys
from typing import List, Optional, Tuple


def _run(cmd: List[str], timeout_sec: int) -> Tuple[int, str, str]:
    proc = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout_sec)
    return proc.returncode, proc.stdout, proc.stderr


def _extract_refpoints_from_echo(text: str) -> List[Tuple[float, float]]:
    refs: List[Tuple[float, float]] = []

    lines = text.splitlines()
    in_refpoint = False
    lat_raw: Optional[int] = None
    lon_raw: Optional[int] = None

    for line in lines:
        stripped = line.strip()

        if stripped.startswith("refPoint:"):
            in_refpoint = True
            lat_raw = None
            lon_raw = None
            continue

        if in_refpoint and re.match(r"^[a-zA-Z_][a-zA-Z0-9_]*:\s*$", stripped):
            # Next top-level/peer field ended refPoint block.
            if lat_raw is not None and lon_raw is not None:
                refs.append((lat_raw * 1e-7, lon_raw * 1e-7))
            in_refpoint = False
            lat_raw = None
            lon_raw = None

        if not in_refpoint:
            continue

        lat_match = re.match(r"^lat:\s*(-?\d+)\s*$", stripped)
        if lat_match:
            lat_raw = int(lat_match.group(1))
            continue

        lon_match = re.match(r"^(long|lon):\s*(-?\d+)\s*$", stripped)
        if lon_match:
            lon_raw = int(lon_match.group(2))
            continue

    if in_refpoint and lat_raw is not None and lon_raw is not None:
        refs.append((lat_raw * 1e-7, lon_raw * 1e-7))

    return refs


def _mean_latlon(points: List[Tuple[float, float]]) -> Tuple[float, float]:
    lat = sum(p[0] for p in points) / len(points)
    lon = sum(p[1] for p in points) / len(points)
    return lat, lon


def main() -> int:
    parser = argparse.ArgumentParser(description="Launch alternate TF using MAP-derived fixed reference")
    parser.add_argument("--map-topic", default="/message/incoming_map", help="Decoded MAP topic")
    parser.add_argument("--timeout-sec", type=int, default=25, help="Seconds to wait for one MAP message")
    parser.add_argument("--alt-launch", default="vehicle_platform system_bringup_tf_alt.launch.py", help="ROS launch target")
    parser.add_argument("--extra-launch-arg", action="append", default=[], help="Extra launch arg key:=value")
    parser.add_argument("--dry-run", action="store_true", help="Print launch command only")
    args = parser.parse_args()

    source_cmd = "source /home/avalocal/ros_drivers/install/setup.bash"
    echo_cmd = f"ros2 topic echo --once {shlex.quote(args.map_topic)}"
    shell_cmd = f"bash -lc {shlex.quote(source_cmd + ' && ' + echo_cmd)}"

    try:
        code, out, err = _run(["bash", "-lc", source_cmd + " && " + echo_cmd], args.timeout_sec)
    except subprocess.TimeoutExpired:
        print(
            f"Timed out waiting for one MAP message on {args.map_topic}. "
            "Ensure decoded MAP publishing is active.",
            file=sys.stderr,
        )
        return 2

    if code != 0:
        print("Failed to read MAP topic.", file=sys.stderr)
        if err:
            print(err.strip(), file=sys.stderr)
        return 2

    refs = _extract_refpoints_from_echo(out)
    if not refs:
        print(
            "Could not extract MAP refPoint lat/lon from echoed message. "
            "Check that /message/incoming_map is decoded MAP data with refPoint fields.",
            file=sys.stderr,
        )
        return 3

    lat_deg, lon_deg = _mean_latlon(refs)

    launch_tokens = args.alt_launch.split()
    if len(launch_tokens) < 2:
        print("--alt-launch must be in format: '<package> <launch_file>'", file=sys.stderr)
        return 4

    package_name = launch_tokens[0]
    launch_file = launch_tokens[1]

    launch_cmd = [
        "ros2",
        "launch",
        package_name,
        launch_file,
        "alt_use_fixed_reference:=true",
        f"alt_fixed_reference_lat_deg:={lat_deg:.8f}",
        f"alt_fixed_reference_lon_deg:={lon_deg:.8f}",
        "alt_fixed_reference_alt_m:=0.0",
    ]
    launch_cmd.extend(args.extra_launch_arg)

    print(f"MAP anchor from {len(refs)} refPoint(s): lat={lat_deg:.8f}, lon={lon_deg:.8f}")
    print("Launch command:")
    print("  " + " ".join(shlex.quote(x) for x in launch_cmd))

    if args.dry_run:
        return 0

    env = os.environ.copy()
    try:
        proc = subprocess.Popen(["bash", "-lc", source_cmd + " && " + " ".join(shlex.quote(x) for x in launch_cmd)], env=env)
        return proc.wait()
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
