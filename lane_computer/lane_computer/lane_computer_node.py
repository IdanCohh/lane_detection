import rclpy
from rclpy.node import Node

from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point

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

        self.trajectory_publisher = self.create_publisher(
            Marker,
            '/assignment/trajectory',
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
        
        if not left_boundary and not right_boundary:
            self.get_logger().error('No lane boundaries found in the message.')
        elif left_boundary and not right_boundary:
            right_boundary = self.interpolate_missing_boundary(left_boundary, offset=-3.5)

        elif right_boundary and not left_boundary:
            left_boundary = self.interpolate_missing_boundary(right_boundary, offset=3.5)

        # else == both lanes data are valid
        
        self.process_lane_boundaries(left_boundary, right_boundary)

    def interpolate_missing_boundary(self, reference_points, offset):
        interpolated_points = []
        for point in reference_points:
            new_point = type(point)()
            new_point.x = point.x
            new_point.y = point.y + offset
            new_point.z = point.z
            interpolated_points.append(new_point)
        return interpolated_points

    def process_lane_boundaries(self, left_points, right_points):
        min_len = min(len(left_points), len(right_points))
        center_points = []
        
        for i in range(min_len):
            center_point = Point()
            center_point.x = (left_points[i].x + right_points[i].x) / 2.0
            center_point.y = (left_points[i].y + right_points[i].y) / 2.0
            center_point.z = (left_points[i].z + right_points[i].z) / 2.0
            center_points.append(center_point)
        
        if center_points:
            self.publish_trajectory(center_points)
    
    def publish_trajectory(self, center_points):
        trajectory = Marker()
        trajectory.header.frame_id = "base_link"
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.ns = "centerline"
        trajectory.id = 0
        trajectory.type = Marker.LINE_STRIP
        trajectory.action = Marker.ADD
        trajectory.points = center_points
        trajectory.scale.x = 0.2
        trajectory.color.a = 1.0
        trajectory.color.r = 1.0
        trajectory.color.g = 0.0
        trajectory.color.b = 0.0
        
        self.trajectory_publisher.publish(trajectory)

def main(args=None):
    rclpy.init(args=args)
    node = LaneComputerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()