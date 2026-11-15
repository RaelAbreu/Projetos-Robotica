import csv
import math

import rclpy
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from nav_msgs.msg import Odometry, Path
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from .ros_params import fparam, param
from .utils import quaternion_from_yaw, wrap_angle, yaw_from_quaternion


class SlamEval(Node):
    def __init__(self):
        super().__init__('slam_eval')
        self.truth_topic = str(param(self, 'truth_topic', '/ground_truth/odom'))
        self.map_frame = str(param(self, 'map_frame', 'map'))
        self.odom_frame = str(param(self, 'odom_frame', 'odom'))
        self.base_frame = str(param(self, 'base_frame', 'chassis'))
        self.log_file = str(param(self, 'log_file', ''))
        self.jump_thr = fparam(self, 'jump_threshold', 0.3)
        self.rec_pos = fparam(self, 'recover_pos_tol', 0.25)
        self.rec_yaw = fparam(self, 'recover_yaw_tol', 0.20)
        self.rec_hold = fparam(self, 'recover_hold', 3.0)
        rate = fparam(self, 'rate', 10.0)

        self.truth = None         
        self.prev_truth = None
        self.t0 = None
        self.kidnap_t = None      
        self.ok_since = None
        self.recovered_at = None
        self.n_kidnaps = 0
        self.sum_sq_slam = 0.0
        self.sum_sq_odom = 0.0
        self.n = 0
        self.max_slam = 0.0

        self.tf_buf = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buf, self)
        self.err_pub = self.create_publisher(Vector3Stamped, 'slam_eval/error', 10)
        self.max_path = int(fparam(self, 'max_path_poses', 4000))
        self.paths = {}
        self.path_pubs = {}
        for key in ('gt', 'slam', 'odom'):
            path = Path()
            path.header.frame_id = self.map_frame
            self.paths[key] = path
            self.path_pubs[key] = self.create_publisher(Path, f'slam_eval/path_{key}', 1)
        self.create_timer(1.0, self.publish_paths)
        self.create_subscription(Odometry, self.truth_topic, self.on_truth, 10)
        self.create_timer(1.0 / rate, self.on_timer)

        self._csv = None
        if self.log_file:
            self._f = open(self.log_file, 'w', newline='')
            self._csv = csv.writer(self._f)
            self._csv.writerow(['t', 'x_gt', 'y_gt', 'th_gt', 'x_slam', 'y_slam', 'th_slam',
                                'x_odom', 'y_odom', 'th_odom',
                                'err_slam_pos', 'err_slam_yaw', 'err_odom_pos', 'err_odom_yaw',
                                'kidnap_t'])
        self.get_logger().info(
            f'avaliação do SLAM vs {self.truth_topic}; log: {self.log_file or "desligado"}')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_truth(self, msg: Odometry):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        cur = (p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))
        if self.prev_truth is not None and \
                math.hypot(cur[0] - self.prev_truth[0], cur[1] - self.prev_truth[1]) > self.jump_thr:
            self.n_kidnaps += 1
            self.kidnap_t = self.now()
            self.ok_since = self.recovered_at = None
            self.get_logger().warn(
                f'SEQUESTRO #{self.n_kidnaps} detectado em t={self.kidnap_t:.1f} s: '
                f'({self.prev_truth[0]:.2f}, {self.prev_truth[1]:.2f}) -> '
                f'({cur[0]:.2f}, {cur[1]:.2f})')
        self.prev_truth = cur
        self.truth = cur

    def lookup(self, parent):
        try:
            tf = self.tf_buf.lookup_transform(parent, self.base_frame, Time())
        except TransformException:
            return None
        t, r = tf.transform.translation, tf.transform.rotation
        return t.x, t.y, yaw_from_quaternion(r.x, r.y, r.z, r.w)

    def add_to_path(self, key, pose, stamp):
        ps = PoseStamped()
        ps.header.stamp, ps.header.frame_id = stamp, self.map_frame
        ps.pose.position.x, ps.pose.position.y = pose[0], pose[1]
        (ps.pose.orientation.x, ps.pose.orientation.y,
         ps.pose.orientation.z, ps.pose.orientation.w) = quaternion_from_yaw(pose[2])
        poses = self.paths[key].poses
        poses.append(ps)
        if len(poses) > self.max_path:
            del poses[0]

    def publish_paths(self):
        stamp = self.get_clock().now().to_msg()
        for key, path in self.paths.items():
            path.header.stamp = stamp
            self.path_pubs[key].publish(path)

    def on_timer(self):
        if self.truth is None:
            return
        slam, odom = self.lookup(self.map_frame), self.lookup(self.odom_frame)
        if slam is None or odom is None:
            return
        now = self.now()
        if self.t0 is None:
            self.t0 = now
        gt = self.truth
        e_slam = math.hypot(slam[0] - gt[0], slam[1] - gt[1])
        e_slam_yaw = abs(wrap_angle(slam[2] - gt[2]))
        e_odom = math.hypot(odom[0] - gt[0], odom[1] - gt[1])
        e_odom_yaw = abs(wrap_angle(odom[2] - gt[2]))

        if self.kidnap_t is None:          # estatística só ANTES de qualquer sequestro
            self.n += 1
            self.sum_sq_slam += e_slam ** 2
            self.sum_sq_odom += e_odom ** 2
            self.max_slam = max(self.max_slam, e_slam)
        elif self.recovered_at is None:
            if e_slam < self.rec_pos and e_slam_yaw < self.rec_yaw:
                self.ok_since = self.ok_since if self.ok_since is not None else now
                if now - self.ok_since >= self.rec_hold:
                    self.recovered_at = self.ok_since
                    self.get_logger().info(
                        f'SLAM RECUPEROU {self.recovered_at - self.kidnap_t:.1f} s após o '
                        f'sequestro (erro {e_slam:.2f} m / {math.degrees(e_slam_yaw):.1f} graus)')
            else:
                self.ok_since = None

        stamp = self.get_clock().now().to_msg()
        self.add_to_path('gt', gt, stamp)
        self.add_to_path('slam', slam, stamp)
        self.add_to_path('odom', odom, stamp)

        msg = Vector3Stamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.map_frame
        msg.vector.x, msg.vector.y, msg.vector.z = e_slam, e_odom, e_slam_yaw
        self.err_pub.publish(msg)

        if self._csv:
            self._csv.writerow([f'{now - self.t0:.3f}', *gt, *slam, *odom,
                                e_slam, e_slam_yaw, e_odom, e_odom_yaw,
                                '' if self.kidnap_t is None else f'{self.kidnap_t - self.t0:.3f}'])
            self._f.flush()

    def summary(self):
        if self.n:
            self.get_logger().info(
                f'RESUMO (antes de qualquer sequestro): RMSE posição SLAM = '
                f'{math.sqrt(self.sum_sq_slam / self.n):.3f} m (máx {self.max_slam:.3f} m) | '
                f'só odometria = {math.sqrt(self.sum_sq_odom / self.n):.3f} m')
        if self.kidnap_t is not None and self.recovered_at is None:
            self.get_logger().warn('o SLAM NÃO se recuperou do sequestro durante a execução')
        if self._csv:
            self._f.close()


def main():
    rclpy.init()
    node = SlamEval()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.summary()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
