import rclpy
from rclpy.node import Node
import tf2_ros

from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point, TwistStamped
from std_msgs.msg import Float64

class LaneComputerNode(Node):
    __slots__ = ('lanes_subscriber', 'trajectory_publisher', 'distance_publisher_tf', 
                 'distance_publisher_vel', 'twist_subscriber', 'tf_buffer', 'tf_listener', 
                 'last_position', 'total_distance_tf', 'total_distance_vel', 'last_time')

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
        
        self.distance_publisher_tf = self.create_publisher(
            Float64,
            '/assignment/travelled_distance_tf',
            10
        )
        
        self.distance_publisher_vel = self.create_publisher(
            Float64,
            '/assignment/travelled_distance_vel',
            10
        )
        
        self.twist_subscriber = self.create_subscription(
            TwistStamped,
            '/vehicle/twist',
            self.twist_callback,
            10
        )
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.last_position = None
        self.total_distance_tf = 0.0
        self.total_distance_vel = 0.0
        self.last_time = None
        
        self.create_timer(0.1, self.update_distance_tf)

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
            return
        
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
    
    def twist_callback(self, msg):
        current_time = self.get_clock().now()
        
        if self.last_time:
            dt = (current_time - self.last_time).nanoseconds / 1e9
            linear_speed = (msg.twist.linear.x**2 + msg.twist.linear.y**2)**0.5
            distance_increment = linear_speed * dt
            self.total_distance_vel += distance_increment
            
            distance_msg = Float64()
            distance_msg.data = self.total_distance_vel
            self.distance_publisher_vel.publish(distance_msg)
        
        self.last_time = current_time
    
    def update_distance_tf(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'odom', 
                'base_link', 
                rclpy.time.Time()
            )
            
            current_position = (
                transform.transform.translation.x,
                transform.transform.translation.y
            )
            
            if self.last_position:
                dx = current_position[0] - self.last_position[0]
                dy = current_position[1] - self.last_position[1]
                distance_increment = (dx*dx + dy*dy)**0.5
                self.total_distance_tf += distance_increment
                
                distance_msg = Float64()
                distance_msg.data = self.total_distance_tf
                self.distance_publisher_tf.publish(distance_msg)
            
            self.last_position = current_position
            
        except Exception as e:
            self.get_logger().error(f"Failed to get transform: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = LaneComputerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()