import rclpy
from .cfs_wrapper import CrazySwarmWrapper

def main():
    rclpy.init()
    cw = CrazySwarmWrapper()
    node = cw.allcfs
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()