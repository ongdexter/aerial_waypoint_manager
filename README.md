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
| `~/abort` | `Trigger` | Emergency stop → IDLE |

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

The generated `.pkl` contains the graph, waypoint GPS coordinates, polygons, satellite image, and pixel/GPS bounds. Set the wildcard `waypoint_data_file` once in the YAML configuration; ROS supplies it to both `waypoint_planner` and `waypoint_gui`. Satellite capture requires Playwright's Chromium browser (`playwright install chromium`).

## GUI Controls

The Qt GUI (`waypoint_gui`) provides:
- **State display**: Color-coded current FSM state
- **GPS display**: Current lat/lon/alt
- **Flight buttons**: TAKEOFF, RTH, HOLD
- **Relative move**: X/Y/Z offset inputs (meters)
