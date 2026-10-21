
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu

from .ros_params import fparam, param


def quat_mul(a, b):
    """Produto de Hamilton, quaternions como (x, y, z, w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([aw * bx + ax * bw + ay * bz - az * by,
                     aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw,
                     aw * bw - ax * bx - ay * by - az * bz])


def rotvec_to_quat(rv):
    angle = float(np.linalg.norm(rv))
    if angle < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0])
    axis = rv / angle
    return np.append(axis * np.sin(angle / 2.0), np.cos(angle / 2.0))


class ImuNoise(Node):
    def __init__(self):
        super().__init__('imu_noise')
        in_topic = str(param(self, 'input_topic', '/imu/data_raw'))
        out_topic = str(param(self, 'output_topic', '/imu/data'))
        self.sigma = fparam(self, 'orientation_std', 0.01) * fparam(self, 'noise_scale', 1.0)
        seed = int(param(self, 'seed', -1))
        self.rng = np.random.default_rng(None if seed < 0 else seed)
        self.pub = self.create_publisher(Imu, out_topic, 10)
        self.create_subscription(Imu, in_topic, self.on_imu, 10)
        self.get_logger().info(
            f'{in_topic} -> {out_topic}, ruído de orientação σ={self.sigma:.4f} rad')

    def on_imu(self, msg: Imu):
        o = msg.orientation
        q = np.array([o.x, o.y, o.z, o.w])
        # pequena rotação aleatória aplicada no referencial do sensor
        q = quat_mul(q, rotvec_to_quat(self.rng.normal(0.0, self.sigma, 3)))
        q /= np.linalg.norm(q)
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = map(float, q)
        var = self.sigma ** 2
        msg.orientation_covariance[0] = msg.orientation_covariance[4] = \
            msg.orientation_covariance[8] = var
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ImuNoise()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
