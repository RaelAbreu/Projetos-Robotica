import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from .control_laws import (ContinuousPoseController, Pose2D,
                           ThreeManeuverPoseController)
from .ros_params import fparam, gains_param, param
from .utils import yaw_from_quaternion


class PoseControllerNode(Node):
    def __init__(self, variant: str = 'continuous'):
        super().__init__('pose_controller' if variant == 'continuous'
                         else 'pose_controller_3m')
        self.variant = variant

        odom_topic = param(self, 'odom_topic', '/diff_drive_controller/odom')
        cmd_topic = param(self, 'cmd_vel_topic',
                          '/diff_drive_controller/cmd_vel_unstamped')
        goal_topic = param(self, 'goal_topic', 'goal_pose')
        rate = fparam(self, 'control_rate', 50.0)
        pos_tol = fparam(self, 'pos_tol', 0.05)
        yaw_tol = fparam(self, 'yaw_tol', 0.05)
        v_max = fparam(self, 'v_max', 0.5)
        w_max = fparam(self, 'w_max', 1.5)

        if variant == 'continuous':
            gains = {
                'rho': gains_param(self, 'rho', 0.8),
                'alpha': gains_param(self, 'alpha', 2.0),
                'beta': gains_param(self, 'beta', -0.5),
                'yaw': gains_param(self, 'yaw', 1.5),
            }
            self.ctrl = ContinuousPoseController(gains, pos_tol, yaw_tol,
                                                 v_max, w_max)
        else:
            gains = {
                'align': gains_param(self, 'align', 1.5),
                'rho': gains_param(self, 'rho', 1.0),
                'heading': gains_param(self, 'heading', 3.0),
                'yaw': gains_param(self, 'yaw', 1.5),
            }
            self.ctrl = ThreeManeuverPoseController(
                gains, pos_tol, yaw_tol, fparam(self, 'align_tol', 0.08),
                v_max, w_max)

        self.pose = None
        self.goal = None
        self.last_t = None

        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.create_subscription(Odometry, odom_topic, self.on_odom, 10)
        self.create_subscription(PoseStamped, goal_topic, self.on_goal, 10)
        self.create_timer(1.0 / rate, self.on_timer)
        self.get_logger().info(
            f'Controle de pose [{variant}] pronto. Publique em /{goal_topic.lstrip("/")}')

    def on_odom(self, msg: Odometry):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.pose = Pose2D(p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))

    def on_goal(self, msg: PoseStamped):
        p, q = msg.pose.position, msg.pose.orientation
        new = Pose2D(p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))
        if self.goal is None or (new.x, new.y, new.yaw) != (
                self.goal.x, self.goal.y, self.goal.yaw):
            self.goal = new
            self.ctrl.reset()
            self.get_logger().info(
                f'Novo objetivo: x={new.x:.2f} y={new.y:.2f} yaw={new.yaw:.2f}')

    def on_timer(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        dt = None if self.last_t is None else now - self.last_t
        self.last_t = now
        if self.pose is None or self.goal is None or not dt or dt <= 0.0:
            return
        v, w, _ = self.ctrl.compute(self.pose, self.goal, min(dt, 0.2))
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = v, w
        self.cmd_pub.publish(cmd)


def _main(variant):
    rclpy.init()
    node = PoseControllerNode(variant)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    _main('continuous')


def main_3m():
    _main('three_maneuvers')
