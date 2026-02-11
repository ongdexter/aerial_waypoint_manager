import os
import pickle
from enum import Enum, auto

import rclpy
from rclpy.node import Node

import networkx as nx
from scipy.spatial import KDTree
from pyproj import Transformer

from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import String, Empty
from std_srvs.srv import Trigger
from geographic_msgs.msg import GeoPoseStamped


class FlightState(Enum):
    """FSM states for the waypoint planner."""
    IDLE = auto()
    TAKEOFF = auto()
    TRACKING = auto()
    RETURNING = auto()
    LANDING = auto()


class WaypointPlannerNode(Node):
    def __init__(self):
        super().__init__('waypoint_planner')

        # Parameters (can be overridden via ros2 param or launch)
        self.declare_parameter('waypoint_graph_file', '')

        # Topics
        self.declare_parameter('uav_gps_topic', '/mavros/global_position/global')
        self.declare_parameter('goal_topic', 'waypoint_request')
        self.declare_parameter('waypoint_response_topic', 'waypoint_response')
        self.declare_parameter('setpoint_topic', '/mavros/setpoint/global')
        self.declare_parameter('land_topic', '~/land')
        self.declare_parameter('state_topic', '~/state')

        # Takeoff parameters
        self.declare_parameter('takeoff_enabled', False)
        self.declare_parameter('use_takeoff_pos', False)
        self.declare_parameter('takeoff_latitude', 0.0)
        self.declare_parameter('takeoff_longitude', 0.0)
        self.declare_parameter('takeoff_altitude', 10.0)

        # FSM parameters
        self.declare_parameter('fsm_rate', 2.0)
        self.declare_parameter('altitude_threshold', 1.0)
        self.declare_parameter('position_threshold', 2.0)
        self.declare_parameter('landing_altitude', 0.5)

        # Load parameters
        waypoint_graph_file = self.get_parameter('waypoint_graph_file').get_parameter_value().string_value

        self.takeoff_enabled = self.get_parameter('takeoff_enabled').get_parameter_value().bool_value
        self.use_takeoff_pos = self.get_parameter('use_takeoff_pos').get_parameter_value().bool_value
        self.takeoff_latitude = self.get_parameter('takeoff_latitude').get_parameter_value().double_value
        self.takeoff_longitude = self.get_parameter('takeoff_longitude').get_parameter_value().double_value
        self.takeoff_altitude = self.get_parameter('takeoff_altitude').get_parameter_value().double_value

        self.fsm_rate = self.get_parameter('fsm_rate').get_parameter_value().double_value
        self.altitude_threshold = self.get_parameter('altitude_threshold').get_parameter_value().double_value
        self.position_threshold = self.get_parameter('position_threshold').get_parameter_value().double_value
        self.landing_altitude = self.get_parameter('landing_altitude').get_parameter_value().double_value

        # Load waypoint graph
        if not os.path.exists(waypoint_graph_file):
            self.get_logger().error(f'Waypoint graph file not found: {waypoint_graph_file}')
            raise FileNotFoundError(waypoint_graph_file)

        with open(waypoint_graph_file, 'rb') as f:
            loaded_data = pickle.load(f)

        if isinstance(loaded_data, dict):
            self.G = loaded_data['graph']
            self.waypoints = loaded_data['waypoints']
            self.get_logger().info(f'Loaded waypoint graph with {self.G.number_of_nodes()} nodes and {self.G.number_of_edges()} edges')
        else:
            self.get_logger().error('Legacy graph format not supported')
            raise ValueError('Please regenerate graph with consolidated format')

        # Coordinate transformer
        self.transformer = Transformer.from_crs("EPSG:4326", "EPSG:32618", always_xy=True)

        # KDTree for nearest waypoint lookups
        waypoints_utm = [self.transformer.transform(lon, lat) for lon, lat in self.waypoints]
        self.kdtree = KDTree(waypoints_utm)

        # FSM state
        self.state = FlightState.IDLE
        self.current_gps = None
        self.home_latitude = None
        self.home_longitude = None
        self.home_altitude = None

        # Path tracking
        self.current_path = None
        self.current_path_idx = 0

        # Current setpoint being published
        self.current_setpoint = None

        # Get topic names
        uav_gps_topic = self.get_parameter('uav_gps_topic').get_parameter_value().string_value
        goal_topic = self.get_parameter('goal_topic').get_parameter_value().string_value
        waypoint_response_topic = self.get_parameter('waypoint_response_topic').get_parameter_value().string_value
        setpoint_topic = self.get_parameter('setpoint_topic').get_parameter_value().string_value
        land_topic = self.get_parameter('land_topic').get_parameter_value().string_value
        state_topic = self.get_parameter('state_topic').get_parameter_value().string_value

        # Publishers
        # Use GeoPoseStamped for setpoints (includes orientation) so we can enforce yaw=0 (north)
        self.setpoint_pub = self.create_publisher(GeoPoseStamped, setpoint_topic, 10)
        # Preview publisher: used to advertise where takeoff would go without commanding the vehicle
        self.preview_pub = self.create_publisher(GeoPoseStamped, 'waypoint_planner/preview_setpoint', 10)
        self.path_pub = self.create_publisher(Path, waypoint_response_topic, 10)
        # GPS-coordinate path for GUI overlay (lat/lon/alt in pose.position x/y/z)
        self.gps_path_pub = self.create_publisher(Path, '~/planned_path_gps', 10)
        self.state_pub = self.create_publisher(String, state_topic, 10)

        # Subscribers
        self.gps_sub = self.create_subscription(NavSatFix, uav_gps_topic, self._on_gps, 10)
        self.goal_sub = self.create_subscription(NavSatFix, goal_topic, self._on_goal, 10)
        self.land_sub = self.create_subscription(Empty, land_topic, self._on_land, 10)

        # Services for FSM control
        self.takeoff_srv = self.create_service(Trigger, '~/takeoff', self._srv_takeoff)
        self.land_srv = self.create_service(Trigger, '~/land_srv', self._srv_land)
        self.abort_srv = self.create_service(Trigger, '~/abort', self._srv_abort)

        # Relative move subscriber (receives X,Y,Z offsets in meters)
        self.relative_move_sub = self.create_subscription(
            Point, '~/relative_move', self._on_relative_move, 10
        )

        # FSM timer
        interval = 1.0 / max(0.1, self.fsm_rate)
        self.fsm_timer = self.create_timer(interval, self._update_fsm)

        self.get_logger().info(f'Waypoint planner FSM started in {self.state.name} state')

    def _publish_state(self):
        """Publish current FSM state."""
        msg = String()
        msg.data = self.state.name
        self.state_pub.publish(msg)

    def _set_state(self, new_state: FlightState):
        """Transition to a new state."""
        if new_state != self.state:
            self.get_logger().info(f'State transition: {self.state.name} -> {new_state.name}')
            self.state = new_state
            self._publish_state()

    def _on_gps(self, msg: NavSatFix):
        """Handle GPS updates."""
        self.current_gps = msg

    def _srv_takeoff(self, request, response):
        """Service callback for takeoff command."""
        if self.state == FlightState.IDLE:
            if self.current_gps is None:
                response.success = False
                response.message = 'No GPS fix available'
            else:
                # Initialize home position
                if self.use_takeoff_pos:
                    self.home_latitude = self.current_gps.latitude
                    self.home_longitude = self.current_gps.longitude
                else:
                    self.home_latitude = self.takeoff_latitude
                    self.home_longitude = self.takeoff_longitude
                self.home_altitude = self.current_gps.altitude
                self.target_takeoff_altitude = self.home_altitude + self.takeoff_altitude
                self._set_gps_setpoint(self.home_latitude, self.home_longitude, self.target_takeoff_altitude)
                self._set_state(FlightState.TAKEOFF)
                response.success = True
                response.message = f'Taking off to {self.target_takeoff_altitude:.1f}m ({self.takeoff_altitude}m above home)'
        else:
            response.success = False
            response.message = f'Cannot takeoff from {self.state.name} state'
        return response

    def _srv_land(self, request, response):
        """Service callback for land command."""
        if self.state == FlightState.TRACKING:
            self._set_state(FlightState.RETURNING)
            response.success = True
            response.message = 'Returning to home and landing'
        elif self.state == FlightState.TAKEOFF:
            self._set_state(FlightState.LANDING)
            response.success = True
            response.message = 'Landing immediately'
        elif self.state == FlightState.RETURNING:
            response.success = True
            response.message = 'Already returning to home'
        elif self.state == FlightState.LANDING:
            response.success = True
            response.message = 'Already landing'
        else:
            response.success = False
            response.message = f'Cannot land from {self.state.name} state'
        return response

    def _srv_abort(self, request, response):
        """Service callback for abort/emergency stop."""
        self.current_path = None
        self.current_path_idx = 0
        self.current_setpoint = None
        self._publish_gps_path([])  # Clear path on GUI
        self._set_state(FlightState.IDLE)
        response.success = True
        response.message = 'Aborted, now in IDLE state'
        return response

    def _on_land(self, msg: Empty):
        """Handle land command."""
        if self.state == FlightState.TRACKING:
            self.get_logger().info('Land command received, returning to home')
            self._set_state(FlightState.RETURNING)
        elif self.state in [FlightState.TAKEOFF, FlightState.IDLE]:
            self.get_logger().info('Land command received during takeoff/idle, landing immediately')
            self._set_state(FlightState.LANDING)
        else:
            self.get_logger().info(f'Land command received but already in {self.state.name}')

    def _on_relative_move(self, msg: Point):
        """Handle relative move command (X, Y, Z offsets in meters)."""
        if self.state not in [FlightState.TRACKING, FlightState.TAKEOFF]:
            self.get_logger().warning(f'Relative move received but in {self.state.name} state, ignoring')
            return

        if self.current_gps is None:
            self.get_logger().error('Relative move received but no GPS fix available')
            return

        # Convert current GPS to UTM
        current_x, current_y = self.transformer.transform(
            self.current_gps.longitude, self.current_gps.latitude
        )

        # Apply offsets (X = East, Y = North, Z = altitude)
        new_x = current_x + msg.x
        new_y = current_y + msg.y
        new_alt = self.current_gps.altitude + msg.z

        # Convert back to GPS
        inv_transformer = Transformer.from_crs("EPSG:32618", "EPSG:4326", always_xy=True)
        new_lon, new_lat = inv_transformer.transform(new_x, new_y)

        # Clear any existing path and set direct setpoint
        self.current_path = None
        self.current_path_idx = 0
        self._set_gps_setpoint(new_lat, new_lon, new_alt)
        
        # Ensure we're in tracking mode
        if self.state == FlightState.TAKEOFF:
            self._set_state(FlightState.TRACKING)

        self.get_logger().info(
            f'Relative move: ({msg.x:.1f}, {msg.y:.1f}, {msg.z:.1f})m -> '
            f'lat={new_lat:.6f}, lon={new_lon:.6f}, alt={new_alt:.1f}m'
        )

    def _on_goal(self, msg: NavSatFix):
        """Handle new goal waypoint request."""
        if self.state not in [FlightState.TRACKING, FlightState.TAKEOFF]:
            self.get_logger().warning(f'Goal received but in {self.state.name} state, ignoring')
            return

        if self.current_gps is None:
            self.get_logger().error('Goal received but no GPS fix available')
            return

        # If in takeoff, transition to tracking
        if self.state == FlightState.TAKEOFF:
            self._set_state(FlightState.TRACKING)

        # Plan path
        start_utm = self.transformer.transform(self.current_gps.longitude, self.current_gps.latitude)
        end_utm = self.transformer.transform(msg.longitude, msg.latitude)

        start_idx = int(self.kdtree.query(start_utm)[1])
        goal_idx = int(self.kdtree.query(end_utm)[1])
        self.get_logger().info(f'Planning path: start_idx={start_idx}, goal_idx={goal_idx}')

        try:
            path = nx.shortest_path(self.G, start_idx, goal_idx, weight='weight')
            self.current_path = list(path)
            self.current_path_idx = 0
            self.get_logger().info(f'Path found with {len(path)} waypoints')

            # Publish path for visualization
            self._publish_path(path)
            self._publish_gps_path(path)

            # Set first waypoint as current setpoint
            if len(self.current_path) > 0:
                self._set_waypoint_setpoint(self.current_path[0])

        except nx.NetworkXNoPath:
            self.get_logger().error('No path found between start and goal')
            self._publish_path([])
            self._publish_gps_path([])

    def _publish_path(self, path):
        """Publish planned path for visualization."""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        poses = []
        for i in path:
            lon, lat = self.waypoints[i]
            x, y = self.transformer.transform(lon, lat)
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.position.z = 0.0
            poses.append(ps)
        path_msg.poses = poses
        self.path_pub.publish(path_msg)

    def _publish_gps_path(self, path):
        """Publish planned path in GPS coordinates for GUI map overlay.
        
        Each pose stores: position.x = latitude, position.y = longitude,
        position.z = altitude (takeoff alt).
        """
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        alt = getattr(self, 'target_takeoff_altitude', 0.0)
        poses = []
        for i in path:
            lon, lat = self.waypoints[i]
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = float(lat)
            ps.pose.position.y = float(lon)
            ps.pose.position.z = float(alt)
            poses.append(ps)
        path_msg.poses = poses
        self.gps_path_pub.publish(path_msg)

    def _set_waypoint_setpoint(self, waypoint_idx: int):
        """Set current setpoint from waypoint index."""
        if waypoint_idx < 0 or waypoint_idx >= len(self.waypoints):
            return
        lon, lat = self.waypoints[waypoint_idx]
        # Fly all waypoints at the same AMSL altitude computed at takeoff
        alt = self.target_takeoff_altitude
        self.current_setpoint = {
            'latitude': lat,
            'longitude': lon,
            'altitude': alt
        }
        self.get_logger().info(f'Setpoint: waypoint {waypoint_idx} -> lat={lat:.6f}, lon={lon:.6f}, alt={alt:.1f}m AMSL')

    def _set_gps_setpoint(self, lat: float, lon: float, alt: float):
        """Set current setpoint from GPS coordinates."""
        self.current_setpoint = {
            'latitude': lat,
            'longitude': lon,
            'altitude': alt
        }

    def _publish_setpoint(self):
        """Publish current setpoint (commands the vehicle) using GeoPoseStamped with north orientation."""
        if self.current_setpoint is None:
            return
        msg = GeoPoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.latitude = float(self.current_setpoint['latitude'])
        msg.pose.position.longitude = float(self.current_setpoint['longitude'])
        msg.pose.position.altitude = float(self.current_setpoint['altitude'])
        # Yaw 0 => quaternion (0,0,0,1)
        msg.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.setpoint_pub.publish(msg)

    def _publish_preview_setpoint(self, lat: float, lon: float, alt: float):
        """Publish a preview setpoint for UI only (does NOT command the vehicle) using GeoPoseStamped."""
        msg = GeoPoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.latitude = float(lat)
        msg.pose.position.longitude = float(lon)
        msg.pose.position.altitude = float(alt)
        msg.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.preview_pub.publish(msg)

    def _distance_to_setpoint(self) -> float:
        """Calculate horizontal distance to current setpoint in meters."""
        if self.current_gps is None or self.current_setpoint is None:
            return float('inf')
        ux, uy = self.transformer.transform(self.current_gps.longitude, self.current_gps.latitude)
        tx, ty = self.transformer.transform(self.current_setpoint['longitude'], self.current_setpoint['latitude'])
        return ((ux - tx) ** 2 + (uy - ty) ** 2) ** 0.5

    def _altitude_error(self) -> float:
        """Calculate altitude error to current setpoint."""
        if self.current_gps is None or self.current_setpoint is None:
            return float('inf')
        return abs(self.current_gps.altitude - self.current_setpoint['altitude'])

    def _update_fsm(self):
        """Main FSM update loop."""
        self._publish_state()

        if self.state == FlightState.IDLE:
            self._handle_idle()
        elif self.state == FlightState.TAKEOFF:
            self._handle_takeoff()
        elif self.state == FlightState.TRACKING:
            self._handle_tracking()
        elif self.state == FlightState.RETURNING:
            self._handle_returning()
        elif self.state == FlightState.LANDING:
            self._handle_landing()

    def _handle_idle(self):
        """Handle IDLE state - publish planned takeoff setpoint for UI preview only."""
        if self.current_gps is None:
            return

        # Preview: compute where takeoff would go based on current GPS
        if self.use_takeoff_pos:
            lat = self.current_gps.latitude
            lon = self.current_gps.longitude
        else:
            lat = self.takeoff_latitude
            lon = self.takeoff_longitude
        alt = self.current_gps.altitude + self.takeoff_altitude

        # Publish preview setpoint (UI only). Do NOT change current_setpoint or command the vehicle.
        self._publish_preview_setpoint(lat, lon, alt)

    def _handle_takeoff(self):
        """Handle TAKEOFF state - ascending to takeoff altitude."""
        self._publish_setpoint()

        if self.current_gps is None:
            return

        # Check if reached takeoff altitude (relative to home)
        alt_error = abs(self.current_gps.altitude - self.target_takeoff_altitude)
        if alt_error <= self.altitude_threshold:
            self.get_logger().info(f'Reached takeoff altitude ({self.current_gps.altitude:.1f}m, {self.takeoff_altitude}m above home)')
            self._set_state(FlightState.TRACKING)

    def _handle_tracking(self):
        """Handle TRACKING state - following waypoint path."""
        self._publish_setpoint()

        if self.current_path is None or len(self.current_path) == 0:
            return
        if self.current_gps is None:
            return
        if self.current_path_idx >= len(self.current_path):
            return

        # Check if reached current waypoint
        dist = self._distance_to_setpoint()
        if dist <= self.position_threshold:
            self.get_logger().info(f'Reached waypoint {self.current_path[self.current_path_idx]} (dist={dist:.2f}m)')
            self.current_path_idx += 1

            if self.current_path_idx < len(self.current_path):
                self._set_waypoint_setpoint(self.current_path[self.current_path_idx])
            else:
                self.get_logger().info('Reached final waypoint of path')
                self.current_path = None
                self.current_path_idx = 0

    def _handle_returning(self):
        """Handle RETURNING state - flying back to home position."""
        # Set home as target at the same takeoff altitude (relative to home)
        self._set_gps_setpoint(self.home_latitude, self.home_longitude, self.target_takeoff_altitude)
        self._publish_setpoint()

        if self.current_gps is None:
            return

        # Check if reached home position
        dist = self._distance_to_setpoint()
        if dist <= self.position_threshold:
            self.get_logger().info(f'Reached home position (dist={dist:.2f}m), landing')
            self._set_state(FlightState.LANDING)

    def _handle_landing(self):
        """Handle LANDING state - return to home at takeoff altitude, then descend."""
        if self.current_gps is None:
            return

        # Phase 1: fly to home at takeoff altitude
        self._set_gps_setpoint(self.home_latitude, self.home_longitude, self.target_takeoff_altitude)
        self._publish_setpoint()

        dist = self._distance_to_setpoint()
        if dist > self.position_threshold:
            # Still en-route to home horizontally
            return

        # Phase 2: overhead home — descend to ground
        landing_alt_amsl = self.home_altitude + self.landing_altitude
        self._set_gps_setpoint(self.home_latitude, self.home_longitude, landing_alt_amsl)
        self._publish_setpoint()

        if self.current_gps.altitude <= landing_alt_amsl + self.altitude_threshold:
            self.get_logger().info('Landed successfully')
            self.current_setpoint = None
            self._set_state(FlightState.IDLE)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
