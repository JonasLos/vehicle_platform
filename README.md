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
| `rear_auto_exposure_min_us` | `100.0` | Rear auto exposure lower limit (µs) |
| `rear_auto_exposure_max_us` | `60000.0` | Rear auto exposure upper limit (µs) |
| `rear_auto_exposure_target` | `50.0` | Rear auto exposure target brightness (0–100) |
| `rear_camera_frame_rate_hz` | `15.0` | Rear camera frame rate |
| `enable_zenohd` | `true` | Start rmw_zenohd |
| `enable_map_odom_tf` | `true` | Enable dynamic `map -> odom` TF from NovAtel odom + GPS |
| `map_use_fixed_reference` | `true` | Keep `map` anchored to fixed global LLA origin |
| `map_fixed_reference_lat_deg` | `30.63789534` | Fixed global map origin latitude |
| `map_fixed_reference_lon_deg` | `-96.47777581` | Fixed global map origin longitude |
| `map_fixed_reference_alt_m` | `55.09027857` | Fixed global map origin altitude (m) |
| `map_anchor_mode` | `gps_fallback` | Anchor behavior (`gps_fallback`, `map_then_lock`, `map_only`) |
| `map_anchor_topic` | `/message/incoming_map` | Decoded MAP topic used when MAP anchor modes are selected |
| `inbound_map_anchor_topic` | `/comms/inbound_binary_msg` | Raw V2X topic used when MAP anchor modes are selected |

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

## Camera topics

Front cameras publish under `/camera_fl/` and `/camera_fr/`.  
Rear cameras publish under `/camera_rear_1/`, `/camera_rear_2/`, `/camera_rear_3/`.

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
