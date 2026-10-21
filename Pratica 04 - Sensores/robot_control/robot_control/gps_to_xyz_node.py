import rclpy
from geometry_msgs.msg import PointStamped
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
        # Deve ser igual ao <heading_deg> do .world (ver enu_to_gazebo_local)
        self.heading_deg = fparam(self, 'world_heading_deg', 180.0)
        self.conv = None
        if not self.use_first_fix:
            # Deve ser o mesmo <spherical_coordinates> do arquivo .world
            self.conv = ENUConverter(fparam(self, 'datum_lat', 38.161479),
                                     fparam(self, 'datum_lon', -122.454630),
                                     fparam(self, 'datum_alt', 0.0))
        self.pub = self.create_publisher(PointStamped, xyz_topic, 10)
        self.create_subscription(NavSatFix, gps_topic, self.on_fix, 10)
        self.get_logger().info(f'{gps_topic} -> {xyz_topic} (frame {self.frame_id})')

    def on_fix(self, msg: NavSatFix):
        if msg.status.status == NavSatStatus.STATUS_NO_FIX:
            return
        if self.conv is None:            # modo "primeiro fix = origem"
            self.conv = ENUConverter(msg.latitude, msg.longitude, msg.altitude)
            self.get_logger().info('Datum definido pelo primeiro fix GPS')
        e, n, u = self.conv.to_enu(msg.latitude, msg.longitude, msg.altitude)
        x, y = enu_to_gazebo_local(e, n, self.heading_deg)   # mesmo frame do mundo/odom
        out = PointStamped()
        out.header.stamp, out.header.frame_id = msg.header.stamp, self.frame_id
        out.point.x, out.point.y, out.point.z = x, y, u
        self.pub.publish(out)


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
