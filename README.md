# waypoint_planner

ROS2 package for UAV waypoint planning with a Finite State Machine (FSM) for flight control. Loads a pre-built waypoint graph and provides path planning, takeoff, tracking, and landing capabilities.

## Features

- **FSM-based flight control**: IDLE → TAKEOFF → TRACKING → RETURNING → LANDING
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
| `RETURNING` | Flying back to home position |
| `LANDING` | Descending to ground |

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/mavros/global_position/global` | `NavSatFix` | UAV GPS input |
| `~/state` | `String` | Current FSM state |
| `waypoint_request` | `NavSatFix` | Goal GPS for path planning |
| `waypoint_response` | `Path` | Planned path output |
| `/mavros/setpoint/global` | `NavSatFix` | Setpoint output |
| `~/relative_move` | `Point` | Relative move (X,Y,Z meters) |

## Services

| Service | Type | Description |
|---------|------|-------------|
| `~/takeoff` | `Trigger` | Start takeoff from IDLE |
| `~/land_srv` | `Trigger` | Return to home and land |
| `~/abort` | `Trigger` | Emergency stop → IDLE |

## Configuration

See `config/waypoint_planner.yaml`:

```yaml
waypoint_planner:
  ros__parameters:
    waypoint_graph_file: '/path/to/waypoint_graph.pkl'
    
    # Takeoff
    takeoff_enabled: true
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

Run waypoint_sampler:

```bash
python waypoint_sampler.py --kml area.kml --resolution 2.0 --output waypoints_data.json
```

## Waypoint Graph

```bash
python waypoint_graph.py --waypoint_data waypoints_data.json --graph_file waypoint_graph.pkl
```

The consolidated `.pkl` file contains both the graph and waypoint GPS coordinates.

## GUI Controls

The Qt GUI (`waypoint_gui`) provides:
- **State display**: Color-coded current FSM state
- **GPS display**: Current lat/lon/alt
- **Flight buttons**: TAKEOFF, LAND, ABORT
- **Relative move**: X/Y/Z offset inputs (meters)
