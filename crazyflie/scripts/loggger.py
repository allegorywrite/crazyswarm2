import rclpy
from rclpy.node import Node

class Logger(Node):
    def __init__(self):
        super().__init__('logger')
        self.declare_parameter('robot_prefix', '/cf1')
        robot_prefix  = self.get_parameter('robot_prefix').value
        
        self.odom_subscription = self.create_subscription(
            Twist,
            robot_prefix + '/odom',
            self.odom_callback,
            10)
        self.cmd_vel_legacy_subscription = self.create_subscription(
            Twist,
            robot_prefix + '/cmd_vel_legacy',
            self.cmd_vel_legacy_callback,
            10)
        
def main(args=None):
    rclpy.init(args=args)
    logger = Logger()
    rclpy.spin(logger)
    logger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()