import rclpy
from rclpy.node import Node

class LaneComputerNode(Node):
    def __init__(self):
        super().__init__('lane_computer_node')

def main(args=None):
    rclpy.init(args=args)
    node = LaneComputerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()