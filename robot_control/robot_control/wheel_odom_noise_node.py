import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from .ros_params import fparam, param


class WheelOdomNoise(Node):
    def __init__(self):
        super().__init__('wheel_odom_noise')
        in_topic = str(param(self, 'input_topic', '/diff_drive_controller/odom'))
        out_topic = str(param(self, 'output_topic', '/wheel_odom'))
        scale = fparam(self, 'noise_scale', 1.0)
        self.std_v = fparam(self, 'linear_std', 0.02) * scale       
        self.std_w = fparam(self, 'angular_std', 0.02) * scale      
        seed = int(param(self, 'seed', -1))
        self.rng = np.random.default_rng(None if seed < 0 else seed)
        sb_v = fparam(self, 'linear_bias_std', 0.0) * scale
        sb_w = fparam(self, 'angular_bias_std', 0.0) * scale
        self.bias_v = float(self.rng.normal(0.0, sb_v)) if sb_v > 0.0 else 0.0   
        self.bias_w = float(self.rng.normal(0.0, sb_w)) if sb_w > 0.0 else 0.0   
        self.pub = self.create_publisher(Odometry, out_topic, 10)
        self.create_subscription(Odometry, in_topic, self.on_odom, 10)
        self.get_logger().info(f'{in_topic} -> {out_topic}, σ_v={self.std_v:.4f} m/s, '
                               f'σ_w={self.std_w:.4f} rad/s')
        if self.bias_v or self.bias_w:
            self.get_logger().info(f'viés sorteado: escala de v = {100 * self.bias_v:+.2f} %, '
                                   f'viés de w = {self.bias_w:+.4f} rad/s '
                                   f'({np.degrees(self.bias_w):+.2f} graus/s)')

    def on_odom(self, msg: Odometry):
        t = msg.twist.twist
        t.linear.x = t.linear.x * (1.0 + self.bias_v) + float(self.rng.normal(0.0, self.std_v))
        t.angular.z = t.angular.z + self.bias_w + float(self.rng.normal(0.0, self.std_w))
        c = msg.twist.covariance                    
        c[0] = max(self.std_v ** 2, 1e-6)
        c[7] = 1e-3                                  
        c[35] = max(self.std_w ** 2, 1e-6)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = WheelOdomNoise()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
