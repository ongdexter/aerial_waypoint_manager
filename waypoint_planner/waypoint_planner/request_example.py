import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

class ExamplePublisher(Node):
    def __init__(self):
        super().__init__('example_pub')
        self.pub = self.create_publisher(NavSatFix, 'waypoint_request', 10)
        self.timer = self.create_timer(1.0, self.timer_cb)
        self.sent = False

    def timer_cb(self):
        if self.sent:
            return
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        # Fill these with a real GPS coordinate near your sampled area (this is the GOAL)
        msg.latitude = 40.0005
        msg.longitude = -73.9995
        msg.altitude = 0.0
        self.pub.publish(msg)
        self.get_logger().info('Published example NavSatFix goal')
        self.sent = True


def main(args=None):
    rclpy.init(args=args)
    node = ExamplePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
