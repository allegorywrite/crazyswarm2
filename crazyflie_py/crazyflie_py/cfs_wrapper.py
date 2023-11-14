from crazyflie_py import *
import math
from .bezier import Node, Segment
from .uav_trajectory import Trajectory
import rclpy
from . import genericJoystick
from .crazyflie import CrazyflieServer, TimeHelper
from std_msgs.msg import String
import re

class CrazySwarmWrapper:
    def __init__(self):
        self.allcfs = CrazyflieServer()
        self.timeHelper = TimeHelper(self.allcfs)
        self.input = genericJoystick.Joystick(self.timeHelper)
        # self.allcfs.sub = self.allcfs.create_subscription(String, '/cmd_code_txt', self.exec_code, 1)

        self.cfnum = len(self.allcfs.crazyflies)
        self.accuracy_distance = 0.1
        self.velocity_average = 2

    def extract_python_code(self, content):
        code_block_regex = re.compile(r"```(.*?)```", re.DOTALL)
        code_blocks = code_block_regex.findall(content)
        if code_blocks:
            full_code = "\n".join(code_blocks)
            if full_code.startswith("python"):
                full_code = full_code[7:]
            return full_code
        else:
            self.allcfs.get_logger().info("No code found.")
            return None

    def exec_code(self, msg):
        print("exec_code")
        self.allcfs.get_logger().info(f"Subscribed Text: {msg.data}")
        code = self.extract_python_code(msg.data)
        if code is not None:
            exec(code)
            # self.allcfs.get_logger().info(f"Exec code: {code}")

    def takeoff(self, drone_id, height):
        duration = height / self.velocity_average
        self.allcfs.crazyflies[drone_id].takeoff(targetHeight=float(height), duration=duration)
        self.allcfs.crazyflies[drone_id].waypoint = [self.allcfs.crazyflies[drone_id].position()[0], self.allcfs.crazyflies[drone_id].position()[1], height]
        self.wait_for_arrive(drone_id, [self.allcfs.crazyflies[drone_id].position()[0], self.allcfs.crazyflies[drone_id].position()[1], height])
        return
    def takeoff_all(self, height):
        duration = height / self.velocity_average
        self.allcfs.takeoff(targetHeight=float(height), duration=duration)
        for i in range(self.cfnum):
            self.allcfs.crazyflies[i].waypoint = [self.allcfs.crazyflies[i].position()[0], self.allcfs.crazyflies[i].position()[1], height]
            self.wait_for_arrive(i, [self.allcfs.crazyflies[i].position()[0], self.allcfs.crazyflies[i].position()[1], height])
        return
    def land(self, drone_id):
        self.wait_for_ready(drone_id)
        height = self.allcfs.crazyflies[drone_id].position()[2]
        duration = height / self.velocity_average
        self.allcfs.crazyflies[drone_id].land(targetHeight=0.04, duration=duration)
        self.allcfs.crazyflies[drone_id].waypoint = [self.allcfs.crazyflies[drone_id].position()[0], self.allcfs.crazyflies[drone_id].position()[1], 0]
        self.wait_for_arrive(drone_id, [self.allcfs.crazyflies[drone_id].position()[0], self.allcfs.crazyflies[drone_id].position()[1], 0])
        return
    def land_all(self):
        # 最も高いドローンの高さを取得
        max_height = 0
        for i in range(self.cfnum):
            self.wait_for_ready(i)
            height = self.allcfs.crazyflies[i].position()[2]
            if max_height < height:
                max_height = height
        duration = max_height / self.velocity_average
        self.allcfs.land(targetHeight=0.04, duration=duration)
        for i in range(self.cfnum):
            self.allcfs.crazyflies[i].waypoint = [self.allcfs.crazyflies[i].position()[0], self.allcfs.crazyflies[i].position()[1], 0]
            self.wait_for_arrive(i, [self.allcfs.crazyflies[i].position()[0], self.allcfs.crazyflies[i].position()[1], 0])
        return
    def get_drone_position(self, drone_id):
        return self.allcfs.crazyflies[drone_id].position()
    def fly_to(self, drone_id, position_absolute):
        self.wait_for_ready(drone_id)
        distance = self.norm(position_absolute - self.allcfs.crazyflies[drone_id].position())
        position = [float(position_absolute[0]), float(position_absolute[1]), float(position_absolute[2])]
        duration = distance / self.velocity_average
        self.allcfs.crazyflies[drone_id].goTo(position, yaw=0.0, duration=duration)
        self.allcfs.crazyflies[drone_id].waypoint = position
        # print("flyto id: ", drone_id, "distance: ", distance)
        # self.accuracy_distance = distance/10
        return
    def fly_to_all(self, position_relative):
        position = [float(position_relative[0]), float(position_relative[1]), float(position_relative[2])]
        duration = self.norm(position) / self.velocity_average
        for i in range(self.cfnum):
            self.wait_for_ready(i)
            self.allcfs.crazyflies[i].waypoint = position + self.allcfs.crazyflies[i].position()
        self.allcfs.goTo(position, yaw=0.0, duration=duration)
        return
    def fly_path(self, drone_id, points):
        segments = []
        segment_time = 0.5
        for i in range(len(points)-1):
            point_from = points[i]
            point_to = points[i+1]
            node_from = Node((point_from[0], point_from[1], point_from[2], 0))
            node_to = Node((point_to[0], point_to[1], point_to[2], 0))
            segments.append(Segment(node_from, node_to, segment_time))
        polynomial = []
        for s in segments:
            polynomial.append(s.get_poly())
        traj1 = Trajectory()
        traj1.set_trajectory(polynomial)
        self.allcfs.crazyflies[drone_id].uploadTrajectory(0, 0, traj1)
        self.timeHelper.sleep(0.1)
        self.allcfs.crazyflies[drone_id].startTrajectory(0)
        return
    def wait_for_ready(self, drone_id):
        while True:
            if not hasattr(self.allcfs.crazyflies[drone_id], 'waypoint'):
                break
            distance = self.norm(self.allcfs.crazyflies[drone_id].waypoint - self.allcfs.crazyflies[drone_id].position())
            # print("waitforready id: ", drone_id, "distance: ", distance)
            if distance < self.accuracy_distance:
                break
            else:
                self.timeHelper.sleep(0.1)
    def wait_for_arrive(self, drone_id, position_absolute):
        while True:
            distance = self.norm(position_absolute - self.allcfs.crazyflies[drone_id].position())
            # print("waitforarrive id: ", drone_id, "distance: ", distance)
            if distance < self.accuracy_distance:
                break
            else:
                self.timeHelper.sleep(0.1)
    def norm(self, vector):
        return math.sqrt(vector[0]**2 + vector[1]**2 + vector[2]**2)









