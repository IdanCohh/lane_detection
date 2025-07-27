import rclpy
from rclpy.node import Node
import tf2_ros
import numpy as np

from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point, TwistStamped
from std_msgs.msg import Float64

class LaneComputerNode(Node):
    __slots__ = ('lanes_subscriber', 'trajectory_publisher', 'distance_publisher_tf', 
                 'distance_publisher_vel', 'twist_subscriber', 'tf_buffer', 'tf_listener', 
                 'last_position', 'total_distance_tf', 'total_distance_vel', 'last_time',
                 'last_left_boundary', 'last_right_boundary')

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
            '/assignment/travelled_distance',
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
        self.last_left_boundary = None
        self.last_right_boundary = None
        
        self.create_timer(0.1, self.update_distance_tf)

    def lanes_callback(self, msg : MarkerArray):
        left_boundary = None
        right_boundary = None

        for marker in msg.markers:
            if marker.text == "L0":
                left_boundary = marker.points
            elif marker.text == "R0":
                right_boundary = marker.points
        
        if left_boundary and right_boundary:
            self.last_left_boundary = left_boundary
            self.last_right_boundary = right_boundary
        elif left_boundary:
            right_boundary = self.last_right_boundary
        elif right_boundary:
            left_boundary = self.last_left_boundary
        else:
            left_boundary = self.last_left_boundary
            right_boundary = self.last_right_boundary

        if not left_boundary or not right_boundary:
            return

        self.process_lane_boundaries(left_boundary, right_boundary)

    def process_lane_boundaries(self, left_points, right_points):
        if len(left_points) > len(right_points):
            longer_lane = left_points
            shorter_lane = right_points
            is_left_longer = True
        else:
            longer_lane = right_points
            shorter_lane = left_points
            is_left_longer = False

        if not shorter_lane or len(shorter_lane) < 2:
            self.publish_trajectory(longer_lane)
            return

        def get_arc_lengths(points):
            points_arr = np.array([[p.x, p.y] for p in points])
            distances = np.sqrt(np.sum(np.diff(points_arr, axis=0)**2, axis=1))
            return np.insert(np.cumsum(distances), 0, 0)

        s_longer = get_arc_lengths(longer_lane)
        s_shorter = get_arc_lengths(shorter_lane)

        if s_shorter[-1] < 1e-6:
            self.publish_trajectory(longer_lane)
            return

        shorter_x = np.array([p.x for p in shorter_lane])
        shorter_y = np.array([p.y for p in shorter_lane])

        interp_x = np.interp(s_longer, s_shorter, shorter_x)
        interp_y = np.interp(s_longer, s_shorter, shorter_y)

        interpolated_shorter_lane = []
        for i in range(len(longer_lane)):
            p = Point()
            p.x = interp_x[i]
            p.y = interp_y[i]
            p.z = longer_lane[i].z
            interpolated_shorter_lane.append(p)

        if is_left_longer:
            left_points = longer_lane
            right_points = interpolated_shorter_lane
        else:
            left_points = interpolated_shorter_lane
            right_points = longer_lane

        center_points = []
        for i in range(len(longer_lane)):
            center_point = Point()
            center_point.x = (left_points[i].x + right_points[i].x) / 2.0
            center_point.y = (left_points[i].y + right_points[i].y) / 2.0
            center_point.z = (left_points[i].z + right_points[i].z) / 2.0
            center_points.append(center_point)
        
        if len(center_points) > 2:
            x_coords = [p.x for p in center_points]
            y_coords = [p.y for p in center_points]
            
            poly = np.poly1d(np.polyfit(x_coords, y_coords, 2))
            
            smoothed_points = []
            for p in center_points:
                sp = Point()
                sp.x = p.x
                sp.y = poly(p.x)
                sp.z = p.z
                smoothed_points.append(sp)
            
            self.publish_trajectory(smoothed_points)
        elif center_points:
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
            pass

def main(args=None):
    rclpy.init(args=args)
    node = LaneComputerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()