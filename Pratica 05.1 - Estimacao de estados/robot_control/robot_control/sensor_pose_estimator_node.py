import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu

from .ros_params import fparam, param
from .utils import quaternion_from_yaw, wrap_angle, yaw_from_quaternion


class SensorPoseEstimator(Node):
    def __init__(self):
        super().__init__('sensor_pose_estimator')
        self.frame_id = str(param(self, 'frame_id', 'map'))
        self.child_frame_id = str(param(self, 'child_frame_id', 'chassis'))
        self.gps_alpha = fparam(self, 'gps_alpha', 1.0)
        self.yaw_alpha = fparam(self, 'yaw_alpha', 1.0)
        gps_topic = str(param(self, 'gps_xyz_topic', '/gps/xyz'))
        imu_topic = str(param(self, 'imu_topic', '/imu/data'))
        out_topic = str(param(self, 'output_topic', '/estimated_odom'))

        self.x = self.y = self.yaw = None
        self.pub = self.create_publisher(Odometry, out_topic, 10)
        self.create_subscription(PointStamped, gps_topic, self.on_gps, 10)
        self.create_subscription(Imu, imu_topic, self.on_imu, 50)
        self.get_logger().info(f'GPS({gps_topic}) + IMU({imu_topic}) -> {out_topic}')

    def on_gps(self, msg: PointStamped):
        px, py = msg.point.x, msg.point.y
        if self.x is None:
            self.x, self.y = px, py
        else:
            a = self.gps_alpha
            self.x += a * (px - self.x)
            self.y += a * (py - self.y)

    def on_imu(self, msg: Imu):
        o = msg.orientation
        yaw = yaw_from_quaternion(o.x, o.y, o.z, o.w)
        if self.yaw is None:
            self.yaw = yaw
        else:
            self.yaw = wrap_angle(self.yaw + self.yaw_alpha * wrap_angle(yaw - self.yaw))
        if self.x is None:                       # ainda sem fix GPS
            return
        out = Odometry()
        out.header.stamp, out.header.frame_id = msg.header.stamp, self.frame_id
        out.child_frame_id = self.child_frame_id
        out.pose.pose.position.x, out.pose.pose.position.y = self.x, self.y
        q = quaternion_from_yaw(self.yaw)
        (out.pose.pose.orientation.x, out.pose.pose.orientation.y,
         out.pose.pose.orientation.z, out.pose.pose.orientation.w) = q
        out.twist.twist.angular.z = msg.angular_velocity.z   # giroscópio (ruidoso)
        self.pub.publish(out)


def main():
    rclpy.init()
    node = SensorPoseEstimator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
