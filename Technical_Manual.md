# econ_docking ROS 2 Package

## Overview
This package implements autonomous battery-based docking for a robot using ArUco marker detection and a simplified docking service protocol.

### Main nodes
- `aruco_marker_detector` — detects ArUco marker from camera image and publishes detection status/offsets.
- `navigate_to_charging_dock_no_nav2` — uses battery state, ArUco detection, lidar, and docking service to navigate, dock, charge, and undock.
- `battery_monitor_and_navigate_docking` — low-battery waypoint navigator; publishes reached_spawn and backs up logs.
- `battery_monitor_and_docking` — battery-aware service manager; starts/stops nav and docking service based on battery, reached_spawn, and docking_charged.

## Behavior summary
1. The nav node watches `battery_info`. If below low threshold, it starts docking.
2. Use ArUco detection and offset to rotate/align.
3. Use lidar range plus approach distance to move toward dock.
4. Send docking service commands (`1`, `11`, `10`, `0`) for dock/undock.
5. Wait for charging status and target battery before undocking.

### Docking flow
1. Battery monitor detects low battery and `reached_spawn` true (or goal reached). Minimum battery threshold is configurable (e.g., 0.30 in `battery_monitor_and_docking.cpp`).
2. Battery manager sends `cmd_vel` stop commands, stops nav service, and starts docking service (`e-condocking.service` / `econ-corridorrun_docking.service`).
3. Docking script runs image publisher and launches `econ_aruco_docking.launch.py`, which starts `aruco_marker_detector` + `navigate_to_charging_dock_no_nav2`.
4. Docking node enters state machine:
   - Search for marker if not detected.
   - Align heading using `aruco_marker_offset` and `angular_velocity_align`.
   - When close enough, execute command sequence to docking station:
     - `cmd=1` enable docking mode,
     - `cmd=11` execute docking,
     - monitor dock pin and charging.
5. On successful dock and charging completed (via battery and `power_supply_status`), docking node executes undocking protocol:
   - `cmd=10` start undocking,
   - `cmd=0` undock,
   - backup by `undocking_distance`.
6. After undocking and battery satisfies threshold, docking node publishes `docking_charged=true`.
7. Battery manager receives `docking_charged` and if battery is no longer low, it stops docking service and restarts nav service.

### Launch file Run
```bash
ros2 launch econ_docking econ_aruco_docking.launch.py
```

### Configurable launch arguments
- `namespace` (default `''`)
- `use_namespace` (default `false`)
- `aruco_calib` (camera calibration YAML)
- `show_image` (debug window; currently unused)
- `image_topic` (default `rgb/image_raw`)
- `target_marker_id` (default `0`)
- `process_every_nth_frame` (default `4`)
- `aruco_dictionary_name` (default `DICT_APRILTAG_36h11`)
- `aruco_marker_side_length` (meters, default `0.09`)
- `staging_waypoints_yaml` (waypoint file)
- `battery_max_threshold` (default `0.93`)
- `battery_min_threshold` (default `0.27`)
- `undocking_distance` (default `0.40`)
- `approach_distance` (default `0.25`)
- `offset_search_tolerance` (default `50`)
- `angular_velocity_search` (default `0.32`)

## Node I/O details

### `battery_monitor_and_navigate_docking` (low-battery waypoint nav)
- **Input topics**:
  - `battery_info` (`sensor_msgs/msg/BatteryState`)
- **Output topics**:
  - `cmd_vel` (`geometry_msgs/msg/Twist`)
  - `reached_spawn` (`std_msgs/msg/Bool`)
- **Action client**:
  - `navigate_to_pose` (`nav2_msgs/action/NavigateToPose`)
- **Parameters**:
  - `yaml_filename` (waypoints YAML)
  - `battery_threshold` (default 0.3)
  - `memlog_src`, `log_src` (log backup paths)
- **Behavior**:
  1. On low battery, cancels previous goals and sends first waypoint from YAML to Nav2.
  2. On successful goal, publishes `reached_spawn=true`, stops robot, backs up logs.
  3. On failure, retries; after max retries publishes `reached_spawn=false`, backs up logs.

### `battery_monitor_and_docking` (battery manager)

- **Service-based Production Flow**

    We use **systemd service scripts** to manage task-level switching between navigation and docking, ensuring efficient resource utilization and faster charging cycles. The services (`econ-corridorrun_base`, `econ-corridorrun_nav`, `econ-corridorrun_docking`) automatically switch operational modes based on battery status.
    This transition logic is handled by the **battery_manager node**, which monitors system state and triggers the appropriate service actions.
    These scripts can be customized based on your application requirements. For detailed implementation, refer to the battery manager node:  
    [`battery_monitor_and_docking.cpp`](packages_ros2/econ_docking/files/econ_docking/src/battery_monitor_and_docking.cpp)

- **Input topics**:
  - `battery_info` (`sensor_msgs/msg/BatteryState`)
  - `reached_spawn` (`std_msgs/msg/Bool`)
  - `docking_charged` (`std_msgs/msg/Bool`)
- **Output topics**:
  - `cmd_vel` (`geometry_msgs/msg/Twist`)
- **Parameters**:
  - `battery_threshold` (default 0.20)
- **Behavior**:
  1. When `battery_info.percentage < battery_threshold`, sets low-battery mode.
  2. On `reached_spawn == true` while low battery, it starts docking worker thread.
  3. Worker stops robot, stops nav service, starts docking service.
  4. On `docking_charged == true` and battery not low, stops docking service and starts nav service.

### `aruco_marker_detector`
- **Input topic**: `sensor_msgs/msg/Image` on configured `image_topic`.
- **Output topics**:
  - `aruco_marker_detected` (`std_msgs/msg/Bool`)
  - `aruco_marker_offset` (`std_msgs/msg/Int32`)
- **Parameters**:
  - `aruco_dictionary_name`
  - `aruco_marker_side_length`
  - `camera_calibration_parameters_filename`
  - `target_marker_id`
  - `process_every_nth_frame`
  - `smoothing_factor`

### `navigate_to_charging_dock_no_nav2`
- **Input topics**:
  - `battery_info` (`sensor_msgs/msg/BatteryState`)
  - `aruco_marker_detected` (`std_msgs/msg/Bool`)
  - `aruco_marker_offset` (`std_msgs/msg/Int32`)
  - `scan_filtered` (`sensor_msgs/msg/LaserScan`)
  - `dock/pin_status` (`std_msgs/msg/Bool`)
- **Output topics**:
  - `cmd_vel` (`geometry_msgs/msg/Twist`)
  - `docking_charged` (`std_msgs/msg/Bool`)
- **Service**: `docking_control` (`ros2_base_interfaces/srv/Dock`)
- **Parameters**:
  - `target_battery`
  - `low_battery_min_threshold`
  - `approach_distance`
  - `undocking_distance`
  - `offset_search_tolerance`
  - `angular_velocity_search`

## Node connection details
- `battery_monitor_and_navigate_docking` publishes `reached_spawn`, used by `battery_monitor_and_docking`.
- `battery_monitor_and_docking` controls systemd services for nav/docking and monitors `docking_charged`.
- `navigate_to_charging_dock_no_nav2` receives `battery_info`, ArUco detection, and scan; outputs `docking_charged`.
- `aruco_marker_detector` provides marker detection output used by `navigate_to_charging_dock_no_nav2`.

## Quick notes
- Ensure `scan_filtered` and `battery_info` are being published by your robot.
- Use a real ArUco marker in view for detection.
- If docking fails, the node uses retry/backoff and backup.
