# waypoint_planner

ROS2 package for UAV waypoint planning with a Finite State Machine (FSM) for flight control. Loads a pre-built waypoint graph and satellite map from one data file and provides path planning, takeoff, tracking, and return-to-home capabilities.

## Features

- **FSM-based flight control**: IDLE → TAKEOFF → TRACKING
- **Path planning**: A* shortest path on pre-built waypoint graph
- **Relative position commands**: Move X/Y/Z meters from current position
- **Qt GUI**: Control panel with state display and relative move controls

## Installation

```bash
# Build
colcon build --symlink-install --packages-select waypoint_planner

# Python dependencies
pip install networkx scipy pyproj PyQt5
```

## Quick Start

```bash
# Terminal 1: Launch the planner
ros2 launch waypoint_planner waypoint_planner.launch.py

# Terminal 2: Launch GUI (optional)
ros2 run waypoint_planner waypoint_gui
```

## Multi-robot MAVROS

The UAV `mavros.launch.py` and UGV `ugv_mavros.launch.py` isolate MAVROS's
internal raw MAVLink topics under `/<robot_ns>/mavros/internal/uas<tgt_system>/`.
Both robots may use FCU system ID `1` without sharing this internal ROS bus.
Public topics retain their existing names, including `/uav/mavros/rc/in` and
`/ugv/rc/in` with the respective default launch namespaces.

Both launch files accept `robot_ns`, `fcu_url`, `system_id` (MAVROS sender ID),
`tgt_system` (actual FCU system ID, default `1`), and `tgt_component` (default `1`).
Sender IDs remain `21` for the UAV and `31` for the UGV. Set `tgt_system` to the
FCU's configured MAVLink ID if it differs from `1`; the old `system_target_id`
parameter was ignored by MAVROS. For example, for an FCU configured with ID `121`:

```bash
ros2 launch waypoint_planner mavros.launch.py tgt_system:=121
```

Rebuild/source the workspace and restart both MAVROS instances to apply the
isolation changes.

## FSM States

| State | Description |
|-------|-------------|
| `IDLE` | Waiting for takeoff command |
| `TAKEOFF` | Ascending to takeoff altitude |
| `TRACKING` | Following waypoints or relative move commands |

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/uav/mavros/global_position/global` | `NavSatFix` | UAV GPS input |
| `~/state` | `String` | Current FSM state |
| `/uav/planner/gps_waypoint` | `NavSatFix` | GPS planning goal (routed through the waypoint graph) |
| `waypoint_response` | `Path` | Planned path output |
| `/uav/waypoint_planner/navigation_status` | `String` | `ACTIVE`, `SUCCEEDED`, or `FAILED` |
| `/uav/waypoint_planner/navigation_feedback` | `Float64` | Remaining planned-path distance in meters |

| `/uav/mavros/setpoint_raw/global` | `GlobalPositionTarget` | Setpoint output |
| `~/relative_move` | `Point` | Relative move (X,Y,Z meters) |

## Services

| Service | Type | Description |
|---------|------|-------------|
| `~/takeoff` | `Trigger` | Start takeoff from IDLE |
| `~/rth` | `Trigger` | Return to home while holding current altitude |
| `~/hold` | `Trigger` | Clear the route and hold current position/altitude in TRACKING |
| `~/abort` | `Trigger` | Abort navigation and hold current position/altitude in TRACKING (same as `~/hold`) |

## Configuration

See `config/waypoint_planner.yaml`:

```yaml
waypoint_planner:
  ros__parameters:
    waypoint_data_file: '/path/to/waypoint_data.pkl'
    
    # Takeoff
    use_takeoff_pos: true  # Use current GPS as home
    takeoff_altitude: 10.0
    
    # FSM
    fsm_rate: 2.0
    altitude_threshold: 1.0
    position_threshold: 2.0
```   

## Waypoint Sampler

Generate polygons using Google Earth:
1. Area of operations (AO) — name this polygon `ao`
2. No-fly zones (NFZ) — name these polygons `nfz_1`, `nfz_2`, etc.
3. Export as KML

Build the sampled points, graph, and GUI satellite image with one command:

```bash
python waypoint_planner/scripts/build_waypoint_data.py --kml area.kml --resolution 2.0 --output waypoint_data.pkl
```

The generated `.pkl` contains the graph, waypoint GPS coordinates, polygons, satellite image, and pixel/GPS bounds. Pass it to the planner launch with `pkl_file:=/path/to/waypoint_data.pkl`. Set `waypoint_data_file` in the YAML configuration for the standalone `waypoint_gui`. Satellite capture requires Playwright's Chromium browser (`playwright install chromium`).

## GUI Controls

The Qt GUI (`waypoint_gui`) provides:
- **State display**: Color-coded current FSM state
- **GPS display**: Current lat/lon/alt
- **Flight buttons**: TAKEOFF, RTH, HOLD
- **Relative move**: X/Y/Z offset inputs (meters)

The single `waypoint_planner.launch.py pkl_file:=/path/to/site.pkl`
explicitly selects the graph. An empty `pkl_file` overrides the YAML file path and
sets `use_waypoint_graph: false`: the node starts without loading a pickle,
manual flight controls remain available, and graph-routed goals report FAILED.
The geographic projection is initialized from the first GPS fix in this mode.
Omitting `pkl_file` also disables graph routing; the launch does not fall back to
the YAML pickle path.
