# marrtino_parameters.py

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter # Not strictly necessary for basic usage, but good for type clarity if needed for set_parameters

class MarrtinoParametersNode(Node):

    def __init__(self):
        super().__init__('marrtino_parameters')
        self.get_logger().info('MARRtino Parameters node has been started.')

        # 1. Declare parameters with default values
        # These defaults will be overridden by the launch file if provided
        self.declare_parameter('robot_name', 'marrtino')
        self.declare_parameter('world', 'empty')
        self.declare_parameter('test', 'test1')


def main(args=None):
    rclpy.init(args=args)
    node = MarrtinoParametersNode()
    rclpy.spin(node) # Keep the node alive to receive potential parameter updates
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

