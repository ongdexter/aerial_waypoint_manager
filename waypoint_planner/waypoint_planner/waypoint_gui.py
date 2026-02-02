#!/usr/bin/env python3
"""
Qt GUI for controlling the Waypoint Planner FSM.
Provides buttons for Takeoff, Land, Abort, and relative position commands.
"""

import sys
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import Point

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QFrame, QGroupBox, QDoubleSpinBox, QGridLayout
)
from PyQt5.QtCore import Qt, pyqtSignal, QObject
from PyQt5.QtGui import QFont


class SignalBridge(QObject):
    """Bridge for thread-safe Qt signal emission from ROS callbacks."""
    state_changed = pyqtSignal(str)
    gps_changed = pyqtSignal(float, float, float)


class WaypointGuiNode(Node):
    """ROS2 node for the waypoint planner GUI."""
    
    def __init__(self, signal_bridge: SignalBridge):
        super().__init__('waypoint_gui')
        self.signal_bridge = signal_bridge
        
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
        
        # Publisher for relative move commands
        self.relative_move_pub = self.create_publisher(
            Point,
            '/waypoint_planner/relative_move',
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


class WaypointGuiWindow(QMainWindow):
    """Main Qt window for waypoint planner control."""
    
    def __init__(self, ros_node: WaypointGuiNode, signal_bridge: SignalBridge):
        super().__init__()
        self.ros_node = ros_node
        self.signal_bridge = signal_bridge
        
        self.setWindowTitle('Waypoint Planner Control')
        self.setMinimumSize(450, 500)
        
        # Connect signals
        self.signal_bridge.state_changed.connect(self._update_state_display)
        self.signal_bridge.gps_changed.connect(self._update_gps_display)
        
        self._setup_ui()
        
        # Current state
        self.current_state = 'UNKNOWN'
    
    def _setup_ui(self):
        """Set up the UI components."""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        layout = QVBoxLayout(central_widget)
        layout.setSpacing(15)
        layout.setContentsMargins(15, 15, 15, 15)
        
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
        
        # Control buttons
        button_group = QGroupBox('Flight Controls')
        button_layout = QHBoxLayout(button_group)
        button_layout.setSpacing(10)
        
        # Takeoff button
        self.takeoff_btn = QPushButton('TAKEOFF')
        self.takeoff_btn.setMinimumHeight(50)
        self.takeoff_btn.setFont(QFont('Arial', 12, QFont.Bold))
        self.takeoff_btn.setStyleSheet(self._button_style('#2e7d32', '#388e3c', '#1b5e20'))
        self.takeoff_btn.clicked.connect(self._on_takeoff)
        button_layout.addWidget(self.takeoff_btn)
        
        # Land button
        self.land_btn = QPushButton('LAND')
        self.land_btn.setMinimumHeight(50)
        self.land_btn.setFont(QFont('Arial', 12, QFont.Bold))
        self.land_btn.setStyleSheet(self._button_style('#1565c0', '#1976d2', '#0d47a1'))
        self.land_btn.clicked.connect(self._on_land)
        button_layout.addWidget(self.land_btn)
        
        # Abort button
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
        
        # Spinboxes for X, Y, Z
        self.x_spin = self._create_spinbox(-100, 100, 0.0, 'X (East)')
        self.y_spin = self._create_spinbox(-100, 100, 0.0, 'Y (North)')
        self.z_spin = self._create_spinbox(-50, 50, 0.0, 'Z (Up)')
        
        move_layout.addWidget(QLabel('X (East):'), 0, 0)
        move_layout.addWidget(self.x_spin, 0, 1)
        move_layout.addWidget(QLabel('Y (North):'), 0, 2)
        move_layout.addWidget(self.y_spin, 0, 3)
        move_layout.addWidget(QLabel('Z (Up):'), 0, 4)
        move_layout.addWidget(self.z_spin, 0, 5)
        
        # Move button
        self.move_btn = QPushButton('SEND MOVE')
        self.move_btn.setMinimumHeight(40)
        self.move_btn.setFont(QFont('Arial', 11, QFont.Bold))
        self.move_btn.setStyleSheet(self._button_style('#6a1b9a', '#7b1fa2', '#4a148c'))
        self.move_btn.clicked.connect(self._on_relative_move)
        move_layout.addWidget(self.move_btn, 1, 0, 1, 6)
        
        layout.addWidget(move_group)
        
        # Status bar
        self.status_label = QLabel('Ready')
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet('color: #888888; padding: 8px;')
        layout.addWidget(self.status_label)
        
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
    
    def _update_gps_display(self, lat: float, lon: float, alt: float):
        """Update GPS display labels."""
        self.lat_label.setText(f'Lat: {lat:.6f}')
        self.lon_label.setText(f'Lon: {lon:.6f}')
        self.alt_label.setText(f'Alt: {alt:.1f}m')
    
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
    
    # Clean shutdown
    executor.shutdown()
    ros_node.destroy_node()
    rclpy.try_shutdown()
    
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
