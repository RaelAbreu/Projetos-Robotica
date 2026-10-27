import rclpy
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus

from .geo import ENUConverter, enu_to_gazebo_local
from .ros_params import fparam, param


class GpsToXyz(Node):
    def __init__(self):
        super().__init__('gps_to_xyz')
        gps_topic = str(param(self, 'gps_topic', '/gps/fix'))
        xyz_topic = str(param(self, 'xyz_topic', '/gps/xyz'))
        self.frame_id = str(param(self, 'frame_id', 'map'))
        self.use_first_fix = bool(param(self, 'use_first_fix_as_datum', False))
        self.heading_deg = fparam(self, 'world_heading_deg', 180.0)
        self.conv = None
        if not self.use_first_fix:
            self.conv = ENUConverter(fparam(self, 'datum_lat', 38.161479),
                                     fparam(self, 'datum_lon', -122.454630),
                                     fparam(self, 'datum_alt', 0.0))
        scale = fparam(self, 'noise_scale', 1.0)
        self.std_h = max(fparam(self, 'position_std', 0.05) * scale, 1e-3)
        self.std_v = max(fparam(self, 'vertical_std', 0.10) * scale, 1e-3)
        self.pub = self.create_publisher(PointStamped, xyz_topic, 10)
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, str(param(self, 'pose_topic', '/gps/pose')), 10)
        self.create_subscription(NavSatFix, gps_topic, self.on_fix, 10)
        self.get_logger().info(f'{gps_topic} -> {xyz_topic} (frame {self.frame_id})')

    def on_fix(self, msg: NavSatFix):
        if msg.status.status == NavSatStatus.STATUS_NO_FIX:
            return
        if self.conv is None:            
            self.conv = ENUConverter(msg.latitude, msg.longitude, msg.altitude)
            self.get_logger().info('Datum definido pelo primeiro fix GPS')
        e, n, u = self.conv.to_enu(msg.latitude, msg.longitude, msg.altitude)
        x, y = enu_to_gazebo_local(e, n, self.heading_deg)   
        out = PointStamped()
        out.header.stamp, out.header.frame_id = msg.header.stamp, self.frame_id
        out.point.x, out.point.y, out.point.z = x, y, u
        self.pub.publish(out)

        pose = PoseWithCovarianceStamped()
        pose.header = out.header
        pose.pose.pose.position.x, pose.pose.pose.position.y = x, y
        pose.pose.pose.position.z = u
        pose.pose.pose.orientation.w = 1.0           
        cov = pose.pose.covariance                  
        cov[0] = cov[7] = self.std_h ** 2
        cov[14] = self.std_v ** 2
        cov[21] = cov[28] = cov[35] = 1e6            
        self.pose_pub.publish(pose)


def main():
    rclpy.init()
    node = GpsToXyz()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
