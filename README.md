# vehicle_platform

ROS 2 (Jazzy) bringup package for the full sensor and driver stack. Provides a single `system_bringup.launch.py` that starts all cameras, LiDARs, GNSS, V2X, DBW, and radar nodes.

## Hardware

| Subsystem | Hardware | Driver |
|---|---|---|
| Front cameras (fl, fr) | AVT Vimba Mako G-319 | `avt_vimba_camera` |
| Rear cameras (rc, tl, tr) | Lucid Arena GigE | `arena_camera_node` |
| LiDAR (×2) | Velodyne | `velodyne_driver` / `velodyne_pointcloud` |
| GNSS/INS | NovAtel OEM7 | `novatel_oem7_driver` |
| V2X OBU | Commsignia | `v2x_ros_driver` |
| DBW | NE Raptor DBW | `raptor_dbw_can` |
| Radar | Delphi ESR | `delphi_esr_driver` |

## Quick start

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch vehicle_platform system_bringup.launch.py
```

## Vehicle selection

`system_bringup.launch.py` dispatches to a per-vehicle bringup. Each vehicle has its own calibration and sensor configs:

| `vehicle` | Bringup | Configs (intrinsics, extrinsics, lidar params, Mako) | Cameras |
|---|---|---|---|
| `blue_jeep` (default) | `launch/vehicles/blue_jeep.launch.py` | `config/` | camera_fl, camera_fr, camera_rear_1..5 |
| `white_jeep` | `launch/vehicles/white_jeep.launch.py` | `config/white_jeep_specific/` | camera_fl, camera_rear_1..3 (rc/tl/tr) |

Select the vehicle per launch, or once per machine:

```bash
ros2 launch vehicle_platform system_bringup.launch.py vehicle:=white_jeep
# or, in ~/.bashrc on that vehicle's computer:
export VEHICLE_PLATFORM_VEHICLE=white_jeep
```

`extrinsics.launch.py` follows the same `vehicle` argument and environment variable. All other launch arguments pass through to the selected bringup. The rest of this README describes the blue jeep bringup. The variants `system_bringup_no_v2x`, `front_cameras_only` and `rear_cameras_only` are blue jeep only.

## Key launch arguments

| Argument | Default | Description |
|---|---|---|
| `enable_camera_fl` | `true` | Front-left AVT camera |
| `enable_camera_fr` | `false` | Front-right AVT camera |
| `enable_front_image_proc` | `true` | Debayer pipeline for front cameras |
| `front_camera_fl_ip` | `192.168.20.10` | Front-left camera IP |
| `front_camera_fr_ip` | `192.168.10.10` | Front-right camera IP |
| `front_camera_params_file` | `config/avt_mako_15hz.yaml` | AVT params override (frame rate, exposure) |
| `rear_camera_1_ip` | `192.168.50.10` | Rear camera 1 IP |
| `rear_camera_2_ip` | `192.168.40.10` | Rear camera 2 IP |
| `rear_camera_3_ip` | `192.168.30.10` | Rear camera 3 IP |
| `rear_auto_exposure` | `true` | Rear camera auto exposure |
| `rear_auto_exposure_min_us` | `10000.0` | Rear auto exposure lower limit (µs) |
| `rear_auto_exposure_max_us` | `100000.0` | Rear auto exposure upper limit (µs). Clamped by the camera to ~66.5 ms at 15 Hz |
| `rear_auto_exposure_target` | `100.0` | Target brightness (camera `TargetBrightness`, integer; camera default 128). Shared by auto exposure and auto gain |
| `rear_auto_gain` | `false` | Rear camera auto gain. Off by default; see the dark-scene note below |
| `rear_auto_gain_min` | `-1.0` | Auto gain lower limit in dB (`-1` leaves the camera's own limit) |
| `rear_auto_gain_max` | `-1.0` | Auto gain upper limit in dB (`-1` leaves the camera's own limit; sensor allows 0–48) |
| `rear_camera_frame_rate_hz` | `15.0` | Rear camera frame rate |
| `rear_camera_use_ros_time` | `true` | Stamp rear images with ROS time on arrival. See the timestamp note below |
| `rear_camera_ptp_enable` | `false` | Enable PTP on the rear cameras (needs a grandmaster) |
| `rear_camera_{1,2,3}_frame_id` | `camera_rear_{1,2,3}` | TF frame per rear camera. **Needs mapping — see below** |
| `rear_camera_{1,2,3}_info_url` | `` | Intrinsics per rear camera. Full `file://`/`package://` URL, absolute path, or a bare filename resolved against `config/`. Empty auto-selects the file matching that camera's `frame_id` |
| `enable_zenohd` | `true` | Start rmw_zenohd |
| `enable_map_odom_tf` | `true` | Enable dynamic `map -> odom` TF from NovAtel odom + GPS |
| `map_use_fixed_reference` | `true` | Keep `map` anchored to fixed global LLA origin |
| `map_fixed_reference_lat_deg` | `30.63789534` | Fixed global map origin latitude |
| `map_fixed_reference_lon_deg` | `-96.47777581` | Fixed global map origin longitude |
| `map_fixed_reference_alt_m` | `55.09027857` | Fixed global map origin altitude (m) |
| `map_anchor_mode` | `gps_fallback` | Anchor behavior (`gps_fallback`, `map_then_lock`, `map_only`) |
| `map_anchor_topic` | `/message/incoming_map` | Decoded MAP topic used when MAP anchor modes are selected |
| `inbound_map_anchor_topic` | `/comms/inbound_binary_msg` | Raw V2X topic used when MAP anchor modes are selected |
| `enable_safety_alert_bridge` | `true` | Enable Python SDK-based V2X safety alert bridge |
| `enable_native_safety_alert_bridge` | `false` | Enable native C++ app-notif V2X safety alert bridge |
| `v2x_log_level` | `WARN` | Log level passed to `v2x_ros_driver` (`DEBUG`, `INFO`, `WARN`, `ERROR`) |
| `v2x_safety_alert_enable_abbrev_overlay` | `true` | Enable RViz 2D overlay text for safety alert abbreviations |
| `v2x_safety_alert_abbrev_overlay_topic` | `/v2x/safety_alert_overlay_text` | RViz 2D overlay topic for active safety alert abbreviation text |
| `v2x_safety_bridge_obu_host` | `192.168.0.54` | OBU host IP for safety bridge connection |
| `v2x_safety_bridge_obu_port` | `43985` | OBU app-notif UDP port used by native C++ bridge |
| `v2x_safety_bridge_local_port` | `0` | Local UDP source port for native bridge client |
| `v2x_safety_bridge_notif_filter_csv` | `` | Optional comma-separated native notif filters (e.g. `FCW,IMA,WWE,WWR`) |

## Config files

| File | Purpose |
|---|---|
| `config/avt_mako_15hz.yaml` | Front camera params: 15 Hz, auto exposure (Continuous, 100–60000 µs) |
| `config/avt_mako_15hz.yaml` | Also applies `ExposureAutoTarget: 50`, `ExposureAutoAlg: Mean` |
| `config/camera_fl.intrinsics.yaml` | Front-left camera intrinsic calibration |
| `config/camera_rc_arena_camera_node.intrinsics.yaml` | Rear-center intrinsics |
| `config/camera_tl_arena_camera_node.intrinsics.yaml` | Rear top-left intrinsics |
| `config/camera_tr_arena_camera_node.intrinsics.yaml` | Rear top-right intrinsics |
| `config/extrinsics.yaml` | Sensor extrinsic transforms |

Rear camera intrinsics are only loaded when the matching
`rear_camera_{1,2,3}_info_url` argument is set; see Camera topics below.

## Camera topics

Front cameras publish under `/camera_fl/` and `/camera_fr/`.  
Rear cameras publish under `/camera_rear_1/`, `/camera_rear_2/`, `/camera_rear_3/`.

Each rear camera publishes `image` and `camera_info`. Full parameter reference
for the rear cameras is in the [`arena_camera_node` README](../arena_camera_node/README.md);
most settings can be changed while streaming:

```bash
ros2 param set /camera_rear_1/arena_camera_node auto_exposure_target 120.0
```

### Rear camera TF frames need mapping

The calibrated frames in `config/extrinsics.yaml` are `camera_rc` (rear centre),
`camera_tl` and `camera_tr` (top left/right), and matching intrinsics files
exist for each. Which physical camera sits behind each IP has **not** been
established — the device user IDs are just `1`, `2`, `3` — so
`rear_camera_{1,2,3}_frame_id` default to `camera_rear_{1,2,3}` and the
`_info_url` arguments default to empty.

That is deliberate: a missing TF frame is a loud, diagnosable error, whereas a
wrong one silently corrupts sensor fusion.

Once the mapping is confirmed, **setting `frame_id` is enough** — the matching
intrinsics file is picked up automatically, so the TF frame and the calibration
cannot disagree:

```bash
ros2 launch vehicle_platform system_bringup.launch.py \
  rear_camera_1_frame_id:=camera_rc \
  rear_camera_2_frame_id:=camera_tl \
  rear_camera_3_frame_id:=camera_tr
```

That resolves each camera to `config/<frame_id>_arena_camera_node.intrinsics.yaml`.
Available files are `camera_rc`, `camera_tl` and `camera_tr`.

To point a camera at a different calibration, `rear_camera_{1,2,3}_info_url`
overrides the automatic choice and accepts any of:

| Form | Example |
|---|---|
| Bare filename, resolved against `config/` | `camera_rc_arena_camera_node.intrinsics.yaml` |
| Absolute path | `/home/avalocal/calib/rear1.yaml` |
| Full URL | `file:///home/avalocal/calib/rear1.yaml`, `package://my_pkg/x.yaml` |

If no calibration is found, launch logs a warning naming what to set, and
`camera_info` is published uncalibrated (no rectification).

### Camera startup failures

If a rear camera is not reachable when its node starts (unplugged, wrong IP, or
still booting), the node aborts rather than retrying. Check the node log before
assuming a configuration problem.

### Timestamps

Rear camera images are stamped with ROS time on arrival. The camera clock
counts from power-on and is unrelated to ROS time, so only set
`rear_camera_use_ros_time:=false` once PTP is enabled *and* locked — otherwise
stamps land in 1970 and break TF, `message_filters` sync, and bag replay.

### Low light

In an unlit space the rear cameras cannot reach any brightness target on
exposure alone; measured mean pixel value was ~0 with auto gain off, versus
~21 with `rear_auto_gain:=true` and `rear_auto_gain_max:=48.0`. Auto gain is
off by default because it adds sensor noise, so enable it for night or indoor
operation:

```bash
ros2 launch vehicle_platform system_bringup.launch.py \
  rear_auto_gain:=true rear_auto_gain_max:=24.0
```

When the camera is pinned at both its exposure and gain ceilings, changing
`rear_auto_exposure_target` has no effect — it is already giving everything it
has.

## Building

```bash
colcon build --packages-select vehicle_platform
```

## TF defaults and global frame behavior

`system_bringup.launch.py` now defaults to a fixed global map anchor so V2X
markers and vehicle pose can share one stable global frame.

Default runtime TF chain:

- `map -> odom` (dynamic, from `map_odom_tf_from_gps.py`)
- `odom -> lidar_tc` (dynamic, from NovAtel odometry)
- `lidar_tc -> base_link` (static identity alias for standard vehicle frame)

This means both `map -> lidar_tc` and `map -> base_link` are available.

If needed, you can still switch to MAP-driven anchoring by overriding:

```bash
map_use_fixed_reference:=false map_anchor_mode:=map_then_lock
```
