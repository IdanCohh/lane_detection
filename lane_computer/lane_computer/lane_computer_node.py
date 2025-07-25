import rclpy
from rclpy.node import Node

from visualization_msgs.msg import MarkerArray

class LaneComputerNode(Node):
    __slots__ = ('lanes_subscriber',)

    def __init__(self):
        super().__init__('lane_computer_node')
        self.get_logger().info('Lane Computer Node has been started.')

        self.lanes_subscriber = self.create_subscription(
            MarkerArray,
            '/vis/lanes',
            self.lanes_callback,
            10
        )

    def lanes_callback(self, msg : MarkerArray):
        left_boundary = None
        right_boundary = None

        for marker in msg.markers:
            if marker.text == "L0":
                left_boundary = marker.points
            elif marker.text == "R0":
                right_boundary = marker.points
        
        if left_boundary and right_boundary:
            self.process_lane_boundaries(left_boundary, right_boundary)

        elif not left_boundary:
            self.get_logger().warn('Left boundary not found.')

        elif not right_boundary:
            self.get_logger().warn('Right boundary not found.')

    def process_lane_boundaries(self, left_points, right_points):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = LaneComputerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()