import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from .ros_params import fparam, param
from .utils import quaternion_from_yaw, wrap_angle, yaw_from_quaternion


class SlamOdom(Node):
    def __init__(self):
        super().__init__('slam_odom')
        self.source = str(param(self, 'source', 'wheels')).lower()
        if self.source not in ('ground_truth', 'wheels'):
            raise ValueError(f"source inválido: {self.source} (use ground_truth ou wheels)")
        gt_topic = str(param(self, 'ground_truth_topic', '/ground_truth/odom'))
        wheel_topic = str(param(self, 'wheel_topic', '/wheel_odom'))
        out_topic = str(param(self, 'output_topic', '/odom_slam'))
        self.odom_frame = str(param(self, 'odom_frame', 'odom'))
        self.base_frame = str(param(self, 'base_frame', 'chassis'))
        self.max_dt = fparam(self, 'max_dt', 0.5)   # descarta saltos de tempo (pausa/reset)

        self.x = self.y = self.yaw = 0.0
        self.last_stamp = None

        self.tf_br = TransformBroadcaster(self)
        self.pub = self.create_publisher(Odometry, out_topic, 10)
        if self.source == 'ground_truth':
            self.create_subscription(Odometry, gt_topic, self.on_ground_truth, 10)
            src_txt = gt_topic
        else:
            self.create_subscription(Odometry, wheel_topic, self.on_wheels, 10)
            src_txt = wheel_topic
        self.get_logger().info(
            f'odometria relativa [{self.source}]: {src_txt} -> TF {self.odom_frame}->'
            f'{self.base_frame} + {out_topic}')

    def on_ground_truth(self, msg: Odometry):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.x, self.y = p.x, p.y
        self.yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self.emit(msg.header.stamp, msg.twist.twist, cov_pos=1e-6, cov_yaw=1e-6)

    def on_wheels(self, msg: Odometry):
        stamp = msg.header.stamp
        t = stamp.sec + stamp.nanosec * 1e-9
        if self.last_stamp is None:
            self.last_stamp = t
            self.emit(stamp, msg.twist.twist, cov_pos=1e-6, cov_yaw=1e-6)
            return
        dt = t - self.last_stamp
        self.last_stamp = t
        if dt <= 0.0 or dt > self.max_dt:        
            return
        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z
        yaw_m = self.yaw + 0.5 * w * dt
        self.x += v * math.cos(yaw_m) * dt
        self.y += v * math.sin(yaw_m) * dt
        self.yaw = wrap_angle(self.yaw + w * dt)
        self.emit(stamp, msg.twist.twist, cov_pos=1e-2, cov_yaw=1e-2)

    def emit(self, stamp, twist, cov_pos, cov_yaw):
        qx, qy, qz, qw = quaternion_from_yaw(self.yaw)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation.x, tf.transform.rotation.y = qx, qy
        tf.transform.rotation.z, tf.transform.rotation.w = qz, qw
        self.tf_br.sendTransform(tf)

        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = self.odom_frame
        od.child_frame_id = self.base_frame
        od.pose.pose.position.x, od.pose.pose.position.y = self.x, self.y
        od.pose.pose.orientation.x, od.pose.pose.orientation.y = qx, qy
        od.pose.pose.orientation.z, od.pose.pose.orientation.w = qz, qw
        od.pose.covariance[0] = od.pose.covariance[7] = cov_pos
        od.pose.covariance[35] = cov_yaw
        od.twist.twist = twist
        self.pub.publish(od)


def main():
    rclpy.init()
    node = SlamOdom()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
