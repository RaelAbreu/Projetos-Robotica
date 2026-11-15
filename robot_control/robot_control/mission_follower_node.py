import csv
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool

from .ros_params import farray, fparam, param
from .utils import quaternion_from_yaw, wrap_angle, yaw_from_quaternion


class MissionFollower(Node):
    def __init__(self):
        super().__init__('mission_follower')
        xs = farray(self, 'waypoints.x', [1.0, 1.0, 0.0])
        ys = farray(self, 'waypoints.y', [0.0, 1.0, 1.0])
        yaws = farray(self, 'waypoints.yaw', [0.0, 1.5708, 3.1416])
        if not (len(xs) == len(ys) == len(yaws)) or len(xs) < 1:
            raise ValueError('waypoints.x/.y/.yaw devem ter o mesmo tamanho (>=1)')
        self.wps = list(zip(xs, ys, yaws))

        self.pos_tol = fparam(self, 'pos_tol', 0.08)
        self.yaw_tol = fparam(self, 'yaw_tol', 0.10)
        self.frame_id = param(self, 'frame_id', 'odom')
        self.log_file = param(self, 'log_file', '')
        self.shutdown_on_finish = bool(param(self, 'shutdown_on_finish', False))
        odom_topic = param(self, 'odom_topic', '/diff_drive_controller/odom')

        self.idx = 0
        self.pose = None
        self.t_start = None
        self.t_wp = None
        self.dist = 0.0
        self.prev_xy = None
        self.done = False
        self.trace = []
        self.finish_at = None

        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.done_pub = self.create_publisher(Bool, 'mission_done', 10)
        self.create_subscription(Odometry, odom_topic, self.on_odom, 10)
        self.create_timer(0.1, self.on_timer)
        self.get_logger().info(f'Missão com {len(self.wps)} waypoints: {self.wps}')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, msg: Odometry):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self.pose = (p.x, p.y, yaw)
        if self.t_start is not None and not self.done:
            if self.prev_xy is not None:
                self.dist += math.hypot(p.x - self.prev_xy[0], p.y - self.prev_xy[1])
            self.prev_xy = (p.x, p.y)
            self.trace.append((self.now() - self.t_start, p.x, p.y, yaw))

    def publish_goal(self):
        x, y, yaw = self.wps[self.idx]
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.frame_id
        m.pose.position.x, m.pose.position.y = x, y
        (m.pose.orientation.x, m.pose.orientation.y,
         m.pose.orientation.z, m.pose.orientation.w) = quaternion_from_yaw(yaw)
        self.goal_pub.publish(m)

    def on_timer(self):
        if self.pose is None:
            return
        if self.done:
            self.done_pub.publish(Bool(data=True))
            if self.shutdown_on_finish and self.now() - self.finish_at > 1.0:
                rclpy.shutdown()
            return
        if self.t_start is None:
            self.t_start = self.t_wp = self.now()
            self.prev_xy = self.pose[:2]
            self.get_logger().info(f'Iniciando waypoint 1/{len(self.wps)}')

        gx, gy, gyaw = self.wps[self.idx]
        x, y, yaw = self.pose
        if (math.hypot(gx - x, gy - y) < self.pos_tol
                and abs(wrap_angle(gyaw - yaw)) < self.yaw_tol):
            t = self.now()
            self.get_logger().info(
                f'Waypoint {self.idx + 1}/{len(self.wps)} alcançado em '
                f'{t - self.t_wp:.1f} s')
            self.t_wp = t
            self.idx += 1
            if self.idx >= len(self.wps):
                self.finish(t)
                return
        self.publish_goal()

    def finish(self, t):
        self.done = True
        self.finish_at = t
        self.get_logger().info(
            f'MISSÃO CONCLUÍDA: tempo total = {t - self.t_start:.1f} s, '
            f'distância percorrida = {self.dist:.2f} m')
        if self.log_file:
            with open(self.log_file, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['t', 'x', 'y', 'yaw'])
                w.writerows(self.trace)
            self.get_logger().info(f'Caminho gravado em {self.log_file}')


def main():
    rclpy.init()
    node = MissionFollower()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
