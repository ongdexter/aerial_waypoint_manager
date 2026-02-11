#!/usr/bin/env python3
"""
Qt GUI for controlling the Waypoint Planner FSM.
Provides buttons for Takeoff, Land, Abort, relative position commands,
satellite map visualization with GPS overlay, and manual waypoint mode.
"""

import sys
import os
import threading

import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Path
from std_msgs.msg import String, UInt32
from geographic_msgs.msg import GeoPoseStamped
from std_srvs.srv import Trigger
from geometry_msgs.msg import Point
from mavros_msgs.msg import State as MavrosState, StatusText

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QFrame, QGroupBox, QDoubleSpinBox, QGridLayout,
    QSplitter, QSizePolicy, QButtonGroup, QRadioButton
)
from PyQt5.QtCore import Qt, pyqtSignal, QObject, QTimer, QPoint
from PyQt5.QtGui import QFont, QPixmap, QImage, QPainter, QPen, QColor, QBrush, QPolygon


class SignalBridge(QObject):
    """Bridge for thread-safe Qt signal emission from ROS callbacks."""
    state_changed = pyqtSignal(str)
    gps_changed = pyqtSignal(float, float, float)
    setpoint_changed = pyqtSignal(float, float, float)  # actual tracked setpoint
    preview_setpoint_changed = pyqtSignal(float, float, float)  # preview only
    mavros_state_changed = pyqtSignal(bool, bool, str, int)  # connected, armed, mode, sys_status
    satellites_changed = pyqtSignal(int)
    statustext_changed = pyqtSignal(int, str)  # severity, text
    path_changed = pyqtSignal(list)  # list of (lat, lon) tuples


class WaypointGuiNode(Node):
    """ROS2 node for the waypoint planner GUI."""
    
    def __init__(self, signal_bridge: SignalBridge):
        super().__init__('waypoint_gui')
        self.signal_bridge = signal_bridge
        
        # Parameters
        self.declare_parameter('satellite_map_file', '')
        self.satellite_map_file = self.get_parameter('satellite_map_file').get_parameter_value().string_value

        # Subscribe to state topic
        self.state_sub = self.create_subscription(
            String,
            '/waypoint_planner/state',
            self._on_state,
            10
        )
        
        # Subscribe to GPS topic
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self._on_gps,
            10
        )

        # Subscribe to current setpoint (tracked waypoint)
        self.setpoint_sub = self.create_subscription(
            GeoPoseStamped,
            '/mavros/setpoint/global',
            self._on_setpoint,
            10
        )

        # Subscribe to preview setpoint (published while IDLE as a UI preview only)
        self.preview_sub = self.create_subscription(
            GeoPoseStamped,
            '/waypoint_planner/preview_setpoint',
            self._on_preview_setpoint,
            10
        )

        # Subscribe to planned GPS path for map overlay
        self.gps_path_sub = self.create_subscription(
            Path,
            '/waypoint_planner/planned_path_gps',
            self._on_gps_path,
            10
        )
        
        # Subscribe to MAVROS state
        self.mavros_state_sub = self.create_subscription(
            MavrosState,
            '/mavros/state',
            self._on_mavros_state,
            10
        )

        # Subscribe to satellite count
        self.sat_sub = self.create_subscription(
            UInt32,
            '/mavros/global_position/raw/satellites',
            self._on_satellites,
            10
        )

        # Subscribe to PX4 status text (preflight errors, warnings, etc.)
        self.statustext_sub = self.create_subscription(
            StatusText,
            '/mavros/statustext/recv',
            self._on_statustext,
            10
        )

        # Publisher for relative move commands
        self.relative_move_pub = self.create_publisher(
            Point,
            '/waypoint_planner/relative_move',
            10
        )

        # Publisher for manual waypoint goals
        self.goal_pub = self.create_publisher(
            NavSatFix,
            '/waypoint_planner/manual_waypoint',
            10
        )

        # Publisher for waypoint mode switches
        self.mode_pub = self.create_publisher(
            String,
            '/waypoint_planner/set_waypoint_mode',
            10
        )
        
        # Service clients
        self.takeoff_client = self.create_client(Trigger, '/waypoint_planner/takeoff')
        self.land_client = self.create_client(Trigger, '/waypoint_planner/land_srv')
        self.abort_client = self.create_client(Trigger, '/waypoint_planner/abort')
        
        self.get_logger().info('Waypoint GUI node started')
    
    def _on_state(self, msg: String):
        """Handle state updates from waypoint planner."""
        self.signal_bridge.state_changed.emit(msg.data)
    
    def _on_gps(self, msg: NavSatFix):
        """Handle GPS updates."""
        self.signal_bridge.gps_changed.emit(msg.latitude, msg.longitude, msg.altitude)

    def _on_setpoint(self, msg: GeoPoseStamped):
        """Handle current setpoint / tracked waypoint updates (these command the vehicle)."""
        self._has_actual_setpoint = True
        lat = msg.pose.position.latitude
        lon = msg.pose.position.longitude
        alt = msg.pose.position.altitude
        self.signal_bridge.setpoint_changed.emit(lat, lon, alt)

    def _on_preview_setpoint(self, msg: GeoPoseStamped):
        """Handle preview-only setpoint updates (IDLE preview)."""
        lat = msg.pose.position.latitude
        lon = msg.pose.position.longitude
        alt = msg.pose.position.altitude
        self.signal_bridge.preview_setpoint_changed.emit(lat, lon, alt)

    def _on_gps_path(self, msg: Path):
        """Handle planned GPS path updates for map overlay."""
        path_points = []
        for ps in msg.poses:
            lat = ps.pose.position.x
            lon = ps.pose.position.y
            path_points.append((lat, lon))
        self.signal_bridge.path_changed.emit(path_points)

    def _on_mavros_state(self, msg: MavrosState):
        """Handle MAVROS state updates."""
        self.signal_bridge.mavros_state_changed.emit(
            msg.connected, msg.armed, msg.mode, msg.system_status
        )

    def _on_satellites(self, msg: UInt32):
        """Handle satellite count updates."""
        self.signal_bridge.satellites_changed.emit(int(msg.data))

    def _on_statustext(self, msg: StatusText):
        """Handle PX4 status text messages (preflight errors, etc.)."""
        self.signal_bridge.statustext_changed.emit(msg.severity, msg.text)
    
    def call_takeoff(self):
        """Call the takeoff service."""
        if not self.takeoff_client.service_is_ready():
            self.get_logger().warning('Takeoff service not available')
            return False, 'Service not available'
        self.takeoff_client.call_async(Trigger.Request())
        return True, 'Takeoff requested'
    
    def call_land(self):
        """Call the land service."""
        if not self.land_client.service_is_ready():
            self.get_logger().warning('Land service not available')
            return False, 'Service not available'
        self.land_client.call_async(Trigger.Request())
        return True, 'Land requested'
    
    def call_abort(self):
        """Call the abort service."""
        if not self.abort_client.service_is_ready():
            self.get_logger().warning('Abort service not available')
            return False, 'Service not available'
        self.abort_client.call_async(Trigger.Request())
        return True, 'Abort requested'
    
    def send_relative_move(self, x: float, y: float, z: float):
        """Send a relative move command."""
        msg = Point()
        msg.x = x
        msg.y = y
        msg.z = z
        self.relative_move_pub.publish(msg)
        return True, f'Move ({x:.1f}, {y:.1f}, {z:.1f})m'

    def send_goal(self, lat: float, lon: float):
        """Publish a manual waypoint goal."""
        msg = NavSatFix()
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = 0.0  # altitude is managed by the planner
        self.goal_pub.publish(msg)
        self.get_logger().info(f'Manual waypoint goal: lat={lat:.6f}, lon={lon:.6f}')
        return True, f'Goal ({lat:.6f}, {lon:.6f})'

    def send_mode(self, mode: str):
        """Publish a waypoint mode switch (AUTO or MANUAL)."""
        msg = String()
        msg.data = mode
        self.mode_pub.publish(msg)
        self.get_logger().info(f'Waypoint mode switch: {mode}')


class MapWidget(QLabel):
    """Widget that displays a satellite image with GPS overlay markers."""

    waypoint_clicked = pyqtSignal(float, float)  # lat, lon

    def __init__(self, pix_gps_map, parent=None):
        super().__init__(parent)
        self.pix_gps_map = pix_gps_map
        self.manual_mode = False

        # Overlay data
        self._gps_pos = None    # (lat, lon)
        self._setpoint = None   # (lat, lon)
        self._preview_sp = None # (lat, lon)
        self._path = []         # [(lat, lon), ...]
        self._manual_wp = None   # (lat, lon) for last manual click
        self._has_actual_setpoint = False

        # Build base pixmap from the satellite image
        img_array = self.pix_gps_map.get_image()
        if img_array.ndim == 3 and img_array.shape[2] == 4:
            fmt = QImage.Format_RGBA8888
        else:
            fmt = QImage.Format_RGB888
        h, w = img_array.shape[:2]
        bytes_per_line = img_array.strides[0]
        self._base_qimage = QImage(img_array.data, w, h, bytes_per_line, fmt)
        self._base_pixmap = QPixmap.fromImage(self._base_qimage)

        self.setMinimumSize(300, 200)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setAlignment(Qt.AlignCenter)
        self.setStyleSheet('background-color: #1a1a1a; border: 1px solid #444444; border-radius: 4px;')
        self._redraw()

    # --- public update methods ---

    def update_gps(self, lat, lon):
        self._gps_pos = (lat, lon)
        self._redraw()

    def update_setpoint(self, lat, lon):
        self._has_actual_setpoint = True
        self._setpoint = (lat, lon)
        self._redraw()

    def update_preview_setpoint(self, lat, lon):
        if not self._has_actual_setpoint:
            self._preview_sp = (lat, lon)
            self._redraw()

    def update_path(self, path_points):
        """path_points: list of (lat, lon)."""
        self._path = list(path_points)
        self._redraw()

    def clear_actual_setpoint(self):
        self._has_actual_setpoint = False
        self._setpoint = None

    def set_manual_waypoint(self, lat, lon):
        self._manual_wp = (lat, lon)
        self._redraw()

    def clear_manual_waypoint(self):
        self._manual_wp = None
        self._redraw()

    # --- coordinate helpers ---

    def _widget_to_image_coords(self, wx, wy):
        """Convert widget pixel coords to original image coords, accounting for scaling."""
        pm = self.pixmap()
        if pm is None:
            return None, None
        # The pixmap is scaled and centered inside this QLabel
        pm_w, pm_h = pm.width(), pm.height()
        lbl_w, lbl_h = self.width(), self.height()
        offset_x = (lbl_w - pm_w) // 2
        offset_y = (lbl_h - pm_h) // 2
        ix = wx - offset_x
        iy = wy - offset_y
        if ix < 0 or iy < 0 or ix >= pm_w or iy >= pm_h:
            return None, None
        # Scale to original image size
        orig_w = self._base_pixmap.width()
        orig_h = self._base_pixmap.height()
        orig_x = int(ix * orig_w / pm_w)
        orig_y = int(iy * orig_h / pm_h)
        return orig_x, orig_y

    def _gps_to_widget(self, lat, lon):
        """Convert GPS to widget pixel coords (for drawing on the scaled pixmap)."""
        px, py = self.pix_gps_map.gps_to_pixel(lat, lon)
        pm = self.pixmap()
        if pm is None:
            return None, None
        scale_x = pm.width() / self._base_pixmap.width()
        scale_y = pm.height() / self._base_pixmap.height()
        return int(px * scale_x), int(py * scale_y)

    # --- mouse events ---

    def mousePressEvent(self, event):
        if not self.manual_mode:
            return
        ix, iy = self._widget_to_image_coords(event.x(), event.y())
        if ix is None:
            return
        lat, lon = self.pix_gps_map.pixel_to_gps(ix, iy)
        self.set_manual_waypoint(lat, lon)
        self.waypoint_clicked.emit(lat, lon)

    # --- rendering ---

    def _redraw(self):
        """Redraw the map with all overlays."""
        # Scale base pixmap to fit widget while keeping aspect ratio
        target_w = self.width() if self.width() > 10 else 600
        target_h = self.height() if self.height() > 10 else 400
        scaled = self._base_pixmap.scaled(target_w, target_h, Qt.KeepAspectRatio, Qt.SmoothTransformation)

        painter = QPainter(scaled)
        painter.setRenderHint(QPainter.Antialiasing)

        scale_x = scaled.width() / self._base_pixmap.width()
        scale_y = scaled.height() / self._base_pixmap.height()

        # Draw AO polygon (cyan, semi-transparent)
        ao_coords = self.pix_gps_map.ao_coords
        if len(ao_coords) >= 3:
            ao_points = []
            for lat, lon in ao_coords:
                px, py = self.pix_gps_map.gps_to_pixel(lat, lon)
                ao_points.append(QPoint(int(px * scale_x), int(py * scale_y)))
            painter.setPen(QPen(QColor(0, 255, 255), 2))
            painter.setBrush(QBrush(QColor(0, 255, 255, 35)))
            painter.drawPolygon(QPolygon(ao_points))

        # Draw NFZ polygons (red, semi-transparent)
        for nfz_coords in self.pix_gps_map.nfz_coords_list:
            if len(nfz_coords) >= 3:
                nfz_points = []
                for lat, lon in nfz_coords:
                    px, py = self.pix_gps_map.gps_to_pixel(lat, lon)
                    nfz_points.append(QPoint(int(px * scale_x), int(py * scale_y)))
                painter.setPen(QPen(QColor(255, 0, 0), 2))
                painter.setBrush(QBrush(QColor(255, 0, 0, 50)))
                painter.drawPolygon(QPolygon(nfz_points))

        # Draw planned path (yellow line)
        if len(self._path) >= 2:
            pen = QPen(QColor(255, 235, 59), 3)
            pen.setStyle(Qt.SolidLine)
            painter.setPen(pen)
            for i in range(len(self._path) - 1):
                lat1, lon1 = self._path[i]
                lat2, lon2 = self._path[i + 1]
                px1, py1 = self.pix_gps_map.gps_to_pixel(lat1, lon1)
                px2, py2 = self.pix_gps_map.gps_to_pixel(lat2, lon2)
                painter.drawLine(
                    int(px1 * scale_x), int(py1 * scale_y),
                    int(px2 * scale_x), int(py2 * scale_y)
                )

        # Draw manual waypoint (orange diamond)
        if self._manual_wp is not None:
            lat, lon = self._manual_wp
            px, py = self.pix_gps_map.gps_to_pixel(lat, lon)
            sx, sy = int(px * scale_x), int(py * scale_y)
            pen = QPen(QColor(255, 152, 0), 2)
            painter.setPen(pen)
            painter.setBrush(QBrush(QColor(255, 152, 0, 180)))
            size = 8
            points = [
                QPoint(sx, sy - size),
                QPoint(sx + size, sy),
                QPoint(sx, sy + size),
                QPoint(sx - size, sy),
            ]
            painter.drawPolygon(QPolygon(points))

        # Draw goal / setpoint marker (green or yellow-green)
        goal = self._setpoint if self._has_actual_setpoint else self._preview_sp
        goal_color = QColor(102, 187, 106) if self._has_actual_setpoint else QColor(153, 153, 51)
        if goal is not None:
            lat, lon = goal
            px, py = self.pix_gps_map.gps_to_pixel(lat, lon)
            sx, sy = int(px * scale_x), int(py * scale_y)
            # Crosshair + circle
            pen = QPen(goal_color, 2)
            painter.setPen(pen)
            painter.setBrush(Qt.NoBrush)
            painter.drawEllipse(QPoint(sx, sy), 10, 10)
            painter.drawLine(sx - 14, sy, sx + 14, sy)
            painter.drawLine(sx, sy - 14, sx, sy + 14)

        # Draw current position (blue dot with white border)
        if self._gps_pos is not None:
            lat, lon = self._gps_pos
            px, py = self.pix_gps_map.gps_to_pixel(lat, lon)
            sx, sy = int(px * scale_x), int(py * scale_y)
            # White border
            painter.setPen(QPen(QColor(255, 255, 255), 2))
            painter.setBrush(QBrush(QColor(33, 150, 243)))
            painter.drawEllipse(QPoint(sx, sy), 7, 7)

        painter.end()
        self.setPixmap(scaled)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._redraw()


class WaypointGuiWindow(QMainWindow):
    """Main Qt window for waypoint planner control."""
    
    def __init__(self, ros_node: WaypointGuiNode, signal_bridge: SignalBridge):
        super().__init__()
        self.ros_node = ros_node
        self.signal_bridge = signal_bridge
        
        self.setWindowTitle('Waypoint Planner Control')
        self.setMinimumSize(1000, 700)
        
        # Connect signals
        self.signal_bridge.state_changed.connect(self._update_state_display)
        self.signal_bridge.gps_changed.connect(self._update_gps_display)
        self.signal_bridge.setpoint_changed.connect(self._update_target_display)
        self.signal_bridge.preview_setpoint_changed.connect(self._update_preview_display)
        self.signal_bridge.mavros_state_changed.connect(self._update_mavros_state)
        self.signal_bridge.satellites_changed.connect(self._update_satellites)
        self.signal_bridge.statustext_changed.connect(self._update_statustext)
        self.signal_bridge.path_changed.connect(self._update_path_display)
        
        # Try to load the satellite map
        self.pix_gps_map = None
        map_file = self.ros_node.satellite_map_file
        if map_file and os.path.exists(map_file):
            try:
                from waypoint_planner.pix_gps_map import PixGpsMap
                self.pix_gps_map = PixGpsMap(map_file)
            except Exception as e:
                print(f'[WaypointGUI] Failed to load satellite map: {e}')

        self._setup_ui()
        
        # Current state
        self.current_state = 'UNKNOWN'
        self._has_actual_setpoint = False

        # Track last received state time and start a timer to detect timeouts
        self.last_state_time = None
        self._state_timeout_s = 3.0
        self._state_timer = QTimer(self)
        self._state_timer.setInterval(1000)
        self._state_timer.timeout.connect(self._check_state_timeout)
        self._state_timer.start()
    
    def _setup_ui(self):
        """Set up the UI components."""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        main_layout.setSpacing(10)
        main_layout.setContentsMargins(10, 10, 10, 10)

        # ====== LEFT PANEL: controls ======
        left_panel = QWidget()
        left_panel.setMaximumWidth(480)
        left_panel.setMinimumWidth(400)
        layout = QVBoxLayout(left_panel)
        layout.setSpacing(12)
        layout.setContentsMargins(5, 5, 5, 5)
        
        # State display
        state_group = QGroupBox('Current State')
        state_layout = QVBoxLayout(state_group)
        
        self.state_label = QLabel('UNKNOWN')
        self.state_label.setAlignment(Qt.AlignCenter)
        self.state_label.setFont(QFont('Arial', 24, QFont.Bold))
        self.state_label.setStyleSheet('''
            QLabel {
                padding: 15px;
                background-color: #2d2d2d;
                color: #888888;
                border-radius: 8px;
            }
        ''')
        state_layout.addWidget(self.state_label)
        layout.addWidget(state_group)
        
        # GPS display
        gps_group = QGroupBox('GPS Position')
        gps_layout = QGridLayout(gps_group)
        
        self.lat_label = QLabel('Lat: --')
        self.lon_label = QLabel('Lon: --')
        self.alt_label = QLabel('Alt: --')
        
        for lbl in [self.lat_label, self.lon_label, self.alt_label]:
            lbl.setFont(QFont('Monospace', 11))
            lbl.setStyleSheet('color: #aaaaaa; padding: 3px;')
        
        gps_layout.addWidget(self.lat_label, 0, 0)
        gps_layout.addWidget(self.lon_label, 0, 1)
        gps_layout.addWidget(self.alt_label, 0, 2)
        layout.addWidget(gps_group)

        # Target waypoint display
        target_group = QGroupBox('Target Waypoint')
        target_layout = QGridLayout(target_group)

        self.target_lat_label = QLabel('Lat: --')
        self.target_lon_label = QLabel('Lon: --')
        self.target_alt_label = QLabel('Alt: --')

        for lbl in [self.target_lat_label, self.target_lon_label, self.target_alt_label]:
            lbl.setFont(QFont('Monospace', 11))
            lbl.setStyleSheet('color: #66bb6a; padding: 3px;')

        target_layout.addWidget(self.target_lat_label, 0, 0)
        target_layout.addWidget(self.target_lon_label, 0, 1)
        target_layout.addWidget(self.target_alt_label, 0, 2)
        layout.addWidget(target_group)

        # PX4 / MAVROS status display
        px4_group = QGroupBox('PX4 Status')
        px4_layout = QGridLayout(px4_group)

        self.connected_label = QLabel('FCU: --')
        self.armed_label = QLabel('Armed: --')
        self.mode_label = QLabel('Mode: --')
        self.sat_label = QLabel('Sats: --')

        for lbl in [self.connected_label, self.armed_label, self.mode_label, self.sat_label]:
            lbl.setFont(QFont('Monospace', 11))
            lbl.setStyleSheet('color: #aaaaaa; padding: 3px;')

        px4_layout.addWidget(self.connected_label, 0, 0)
        px4_layout.addWidget(self.armed_label, 0, 1)
        px4_layout.addWidget(self.mode_label, 0, 2)
        px4_layout.addWidget(self.sat_label, 0, 3)

        self.statustext_label = QLabel('PX4: --')
        self.statustext_label.setFont(QFont('Monospace', 10))
        self.statustext_label.setWordWrap(True)
        self.statustext_label.setStyleSheet('color: #aaaaaa; padding: 3px;')
        px4_layout.addWidget(self.statustext_label, 1, 0, 1, 4)

        layout.addWidget(px4_group)

        # Control buttons
        button_group = QGroupBox('Flight Controls')
        button_layout = QHBoxLayout(button_group)
        button_layout.setSpacing(10)
        
        self.takeoff_btn = QPushButton('TAKEOFF')
        self.takeoff_btn.setMinimumHeight(50)
        self.takeoff_btn.setFont(QFont('Arial', 12, QFont.Bold))
        self.takeoff_btn.setStyleSheet(self._button_style('#2e7d32', '#388e3c', '#1b5e20'))
        self.takeoff_btn.clicked.connect(self._on_takeoff)
        button_layout.addWidget(self.takeoff_btn)
        
        self.land_btn = QPushButton('LAND')
        self.land_btn.setMinimumHeight(50)
        self.land_btn.setFont(QFont('Arial', 12, QFont.Bold))
        self.land_btn.setStyleSheet(self._button_style('#1565c0', '#1976d2', '#0d47a1'))
        self.land_btn.clicked.connect(self._on_land)
        button_layout.addWidget(self.land_btn)
        
        self.abort_btn = QPushButton('ABORT')
        self.abort_btn.setMinimumHeight(50)
        self.abort_btn.setFont(QFont('Arial', 12, QFont.Bold))
        self.abort_btn.setStyleSheet(self._button_style('#c62828', '#d32f2f', '#b71c1c'))
        self.abort_btn.clicked.connect(self._on_abort)
        button_layout.addWidget(self.abort_btn)
        
        layout.addWidget(button_group)
        
        # Relative move controls
        move_group = QGroupBox('Relative Move (meters)')
        move_layout = QGridLayout(move_group)
        move_layout.setSpacing(8)
        
        self.x_spin = self._create_spinbox(-100, 100, 0.0, 'X (East)')
        self.y_spin = self._create_spinbox(-100, 100, 0.0, 'Y (North)')
        self.z_spin = self._create_spinbox(-50, 50, 0.0, 'Z (Up)')
        
        move_layout.addWidget(QLabel('X (East):'), 0, 0)
        move_layout.addWidget(self.x_spin, 0, 1)
        move_layout.addWidget(QLabel('Y (North):'), 0, 2)
        move_layout.addWidget(self.y_spin, 0, 3)
        move_layout.addWidget(QLabel('Z (Up):'), 0, 4)
        move_layout.addWidget(self.z_spin, 0, 5)
        
        self.move_btn = QPushButton('SEND MOVE')
        self.move_btn.setMinimumHeight(40)
        self.move_btn.setFont(QFont('Arial', 11, QFont.Bold))
        self.move_btn.setStyleSheet(self._button_style('#6a1b9a', '#7b1fa2', '#4a148c'))
        self.move_btn.clicked.connect(self._on_relative_move)
        move_layout.addWidget(self.move_btn, 1, 0, 1, 6)
        
        layout.addWidget(move_group)
        
        # Status bar
        self.status_label = QLabel('Waiting for connection...')
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet('color: #888888; padding: 8px;')
        layout.addWidget(self.status_label)

        layout.addStretch()

        main_layout.addWidget(left_panel)

        # ====== RIGHT PANEL: map ======
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        right_layout.setSpacing(8)
        right_layout.setContentsMargins(5, 5, 5, 5)

        # Mode toggle
        mode_group = QGroupBox('Waypoint Mode')
        mode_layout = QHBoxLayout(mode_group)

        self.autonomy_radio = QRadioButton('Autonomy')
        self.manual_radio = QRadioButton('Manual Waypoint')
        self.autonomy_radio.setChecked(True)
        self.autonomy_radio.setFont(QFont('Arial', 11))
        self.manual_radio.setFont(QFont('Arial', 11))

        self.mode_btn_group = QButtonGroup(self)
        self.mode_btn_group.addButton(self.autonomy_radio, 0)
        self.mode_btn_group.addButton(self.manual_radio, 1)
        self.mode_btn_group.buttonClicked.connect(self._on_mode_changed)

        mode_layout.addWidget(self.autonomy_radio)
        mode_layout.addWidget(self.manual_radio)

        right_layout.addWidget(mode_group)

        # Map widget
        map_group = QGroupBox('Map View')
        map_layout = QVBoxLayout(map_group)

        if self.pix_gps_map is not None:
            self.map_widget = MapWidget(self.pix_gps_map)
            self.map_widget.waypoint_clicked.connect(self._on_map_waypoint_click)
            map_layout.addWidget(self.map_widget)
        else:
            self.map_widget = None
            no_map_label = QLabel('No satellite map loaded.\n\nSet satellite_map_file parameter\nto a .pkl file generated by\nextract_image_data.py')
            no_map_label.setAlignment(Qt.AlignCenter)
            no_map_label.setFont(QFont('Monospace', 11))
            no_map_label.setStyleSheet('color: #888888; padding: 40px;')
            map_layout.addWidget(no_map_label)

        right_layout.addWidget(map_group)

        # Map legend
        legend_label = QLabel(
            '<span style="color:#00ffff;">▢ AO</span> &nbsp; '
            '<span style="color:#ff0000;">▢ NFZ</span> &nbsp; '
            '<span style="color:#2196f3;">● Position</span> &nbsp; '
            '<span style="color:#66bb6a;">⊕ Goal</span> &nbsp; '
            '<span style="color:#ffeb3b;">━ Path</span> &nbsp; '
            '<span style="color:#ff9800;">◆ Manual WP</span>'
        )
        legend_label.setFont(QFont('Arial', 10))
        legend_label.setAlignment(Qt.AlignCenter)
        legend_label.setStyleSheet('color: #aaaaaa; padding: 4px;')
        right_layout.addWidget(legend_label)

        main_layout.addWidget(right_panel, 1)  # stretch factor 1 for the map

        # Apply dark theme
        self.setStyleSheet('''
            QMainWindow {
                background-color: #1e1e1e;
            }
            QGroupBox {
                color: #cccccc;
                font-size: 12px;
                font-weight: bold;
                border: 1px solid #444444;
                border-radius: 6px;
                margin-top: 8px;
                padding-top: 8px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 8px;
                padding: 0 4px;
            }
            QLabel {
                color: #cccccc;
            }
            QDoubleSpinBox {
                background-color: #2d2d2d;
                color: #ffffff;
                border: 1px solid #444444;
                border-radius: 4px;
                padding: 4px;
            }
            QRadioButton {
                color: #cccccc;
                spacing: 6px;
            }
            QRadioButton::indicator {
                width: 14px;
                height: 14px;
            }
        ''')
    
    def _button_style(self, normal: str, hover: str, pressed: str) -> str:
        return f'''
            QPushButton {{
                background-color: {normal};
                color: white;
                border: none;
                border-radius: 6px;
            }}
            QPushButton:hover {{
                background-color: {hover};
            }}
            QPushButton:pressed {{
                background-color: {pressed};
            }}
            QPushButton:disabled {{
                background-color: #555555;
                color: #888888;
            }}
        '''
    
    def _create_spinbox(self, min_val: float, max_val: float, default: float, tooltip: str) -> QDoubleSpinBox:
        spin = QDoubleSpinBox()
        spin.setRange(min_val, max_val)
        spin.setValue(default)
        spin.setSingleStep(1.0)
        spin.setDecimals(1)
        spin.setToolTip(tooltip)
        spin.setMinimumWidth(70)
        return spin
    
    def _update_state_display(self, state: str):
        """Update the state display label."""
        self.last_state_time = time.time()
        self.status_label.setText('Ready')
        self.current_state = state
        self.state_label.setText(state)
        
        colors = {
            'IDLE': ('#888888', '#2d2d2d'),
            'TAKEOFF': ('#4caf50', '#1b3d1f'),
            'TRACKING': ('#2196f3', '#0d3251'),
            'RETURNING': ('#ff9800', '#4d2e00'),
            'LANDING': ('#9c27b0', '#3d0f47'),
        }
        
        fg, bg = colors.get(state, ('#888888', '#2d2d2d'))
        self.state_label.setStyleSheet(f'''
            QLabel {{
                padding: 15px;
                background-color: {bg};
                color: {fg};
                border-radius: 8px;
            }}
        ''')
        
        # Update button states
        self.takeoff_btn.setEnabled(state == 'IDLE')
        self.land_btn.setEnabled(state in ['TAKEOFF', 'TRACKING'])
        self.move_btn.setEnabled(state in ['TAKEOFF', 'TRACKING'])

        # Reset actual setpoint flag when entering IDLE
        if state == 'IDLE':
            self._has_actual_setpoint = False
            if self.map_widget:
                self.map_widget.clear_actual_setpoint()
    
    def _update_gps_display(self, lat: float, lon: float, alt: float):
        """Update GPS display labels and map."""
        self.lat_label.setText(f'Lat: {lat:.6f}')
        self.lon_label.setText(f'Lon: {lon:.6f}')
        self.alt_label.setText(f'Alt: {alt:.1f}m')
        if self.map_widget:
            self.map_widget.update_gps(lat, lon)

    def _check_state_timeout(self):
        """Check for state message timeout and update status_label accordingly."""
        if self.last_state_time is None:
            if self.status_label.text() != 'Waiting for connection...':
                self.status_label.setText('Waiting for connection...')
            return
        if time.time() - self.last_state_time > self._state_timeout_s:
            if self.status_label.text() != 'Waiting for connection...':
                self.status_label.setText('Waiting for connection...')
        else:
            if self.status_label.text() == 'Waiting for connection...':
                self.status_label.setText('Ready')

    def _update_target_display(self, lat: float, lon: float, alt: float):
        """Update target waypoint display for an actual tracked setpoint (commands vehicle)."""
        self._has_actual_setpoint = True
        self.target_lat_label.setText(f'Lat: {lat:.6f}')
        self.target_lon_label.setText(f'Lon: {lon:.6f}')
        self.target_alt_label.setText(f'Alt: {alt:.1f}m')
        for lbl in [self.target_lat_label, self.target_lon_label, self.target_alt_label]:
            lbl.setStyleSheet('color: #66bb6a; padding: 3px;')
        if self.map_widget:
            self.map_widget.update_setpoint(lat, lon)

    def _update_preview_display(self, lat: float, lon: float, alt: float):
        """Update target waypoint display for a preview setpoint only when no actual setpoint is active."""
        if self._has_actual_setpoint:
            return
        self.target_lat_label.setText(f'Lat: {lat:.6f}')
        self.target_lon_label.setText(f'Lon: {lon:.6f}')
        self.target_alt_label.setText(f'Alt: {alt:.1f}m')
        for lbl in [self.target_lat_label, self.target_lon_label, self.target_alt_label]:
            lbl.setStyleSheet('color: #999933; padding: 3px;')
        if self.map_widget:
            self.map_widget.update_preview_setpoint(lat, lon)

    def _update_path_display(self, path_points: list):
        """Update planned path on the map."""
        if self.map_widget:
            self.map_widget.update_path(path_points)

    def _update_mavros_state(self, connected: bool, armed: bool, mode: str, sys_status: int):
        """Update MAVROS / FCU state display."""
        conn_txt = 'Connected' if connected else 'Disconnected'
        conn_color = '#4caf50' if connected else '#f44336'
        self.connected_label.setText(f'FCU: {conn_txt}')
        self.connected_label.setStyleSheet(f'color: {conn_color}; padding: 3px;')

        arm_txt = 'ARMED' if armed else 'Disarmed'
        arm_color = '#ff9800' if armed else '#aaaaaa'
        self.armed_label.setText(f'Armed: {arm_txt}')
        self.armed_label.setStyleSheet(f'color: {arm_color}; padding: 3px;')

        self.mode_label.setText(f'Mode: {mode}')

    def _update_satellites(self, count: int):
        """Update satellite count display."""
        color = '#4caf50' if count >= 10 else '#ff9800' if count >= 6 else '#f44336'
        self.sat_label.setText(f'Sats: {count}')
        self.sat_label.setStyleSheet(f'color: {color}; padding: 3px;')

    def _update_statustext(self, severity: int, text: str):
        """Update PX4 status text display. Severity: 0-3 error/warning, 4+ info."""
        if severity <= 3:
            color = '#f44336'
        elif severity <= 5:
            color = '#ff9800'
        else:
            color = '#aaaaaa'
        self.statustext_label.setText(f'PX4: {text}')
        self.statustext_label.setStyleSheet(f'color: {color}; padding: 3px;')

    def _on_takeoff(self):
        success, msg = self.ros_node.call_takeoff()
        self.status_label.setText(msg)
    
    def _on_land(self):
        success, msg = self.ros_node.call_land()
        self.status_label.setText(msg)
    
    def _on_abort(self):
        success, msg = self.ros_node.call_abort()
        self.status_label.setText(msg)
    
    def _on_relative_move(self):
        x = self.x_spin.value()
        y = self.y_spin.value()
        z = self.z_spin.value()
        success, msg = self.ros_node.send_relative_move(x, y, z)
        self.status_label.setText(msg)

    def _on_mode_changed(self, button):
        """Handle mode toggle between Autonomy and Manual Waypoint."""
        is_manual = self.manual_radio.isChecked()
        if self.map_widget:
            self.map_widget.manual_mode = is_manual
            if not is_manual:
                self.map_widget.clear_manual_waypoint()
        # Notify the planner node of the mode switch
        self.ros_node.send_mode('MANUAL' if is_manual else 'AUTO')
        mode_name = 'Manual Waypoint' if is_manual else 'Autonomy'
        self.status_label.setText(f'Mode: {mode_name}')

    def _on_map_waypoint_click(self, lat: float, lon: float):
        """Handle a manual waypoint click on the map."""
        success, msg = self.ros_node.send_goal(lat, lon)
        self.status_label.setText(f'Manual WP: {lat:.6f}, {lon:.6f}')




def main(args=None):
    rclpy.init(args=args)
    
    signal_bridge = SignalBridge()
    ros_node = WaypointGuiNode(signal_bridge)
    
    executor = MultiThreadedExecutor()
    executor.add_node(ros_node)
    
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()
    
    app = QApplication(sys.argv)
    window = WaypointGuiWindow(ros_node, signal_bridge)
    window.show()
    
    exit_code = app.exec_()    
    
    executor.shutdown()
    ros_node.destroy_node()
    rclpy.try_shutdown()
    
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
