#!/usr/bin/env python3

import rclpy
from crazyflie_py.cfs_wrapper import CrazySwarmWrapper
import re
from std_msgs.msg import String

class Interpreter:
    def __init__(self) -> None:
        self.cw = CrazySwarmWrapper()
        self.node = self.cw.allcfs
        self.node.get_logger().info(f"Hello World: {0}")
        self.node.sub = self.node.create_subscription(String, '/cmd_code_txt', self.exec_code, 1)

    def extract_python_code(self, content):
        code_block_regex = re.compile(r"```(.*?)```", re.DOTALL)
        code_blocks = code_block_regex.findall(content)
        if code_blocks:
            full_code = "\n".join(code_blocks)
            if full_code.startswith("python"):
                full_code = full_code[7:]
            return full_code
        else:
            self.node.get_logger().info("No code found.")
            return None

    def exec_code(self, msg):
        print("exec_code")
        self.node.get_logger().info(f"Subscribed Text: {msg.data}")
        code = self.extract_python_code(msg.data)
        cw = self.cw
        if code is not None:
            exec(code)
            # self.allcfs.get_logger().info(f"Exec code: {code}")

def main():
    rclpy.init()
    interpreter = Interpreter()
    rclpy.spin(interpreter.node)
    interpreter.node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()