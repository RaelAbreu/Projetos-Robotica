import csv
import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist, Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from .control_laws import Pose2D, RMSE, TrajectoryTracker, tracking_error
from .ros_params import fparam, gains_param, param
from .trajectory import Figure8
from .utils import quaternion_from_yaw, yaw_from_quaternion


class TrajectoryFollower(Node):
    def __init__(self):
        super().__init__('trajectory_follower')
        self.mode = str(param(self, 'mode', 'feedback'))
        self.k_ff = fparam(self, 'k_ff', 1.0)
        traj = Figure8(fparam(self, 'A', 1.0), fparam(self, 'B', 2.0), fparam(self, 'omega', 0.5))
        gains = {
            'x': gains_param(self, 'x', 2.0, 0.3, 0.0),
            'y': gains_param(self, 'y', 2.5),
            'theta': gains_param(self, 'theta', 3.0),
        }
        self.tracker = TrajectoryTracker(
            traj, self.mode, self.k_ff, gains,
            fparam(self, 'v_max', 0.5), fparam(self, 'w_max', 1.5))

        self.duration = fparam(self, 'n_periods', 1.0) * traj.period
        self.start_delay = fparam(self, 'start_delay', 1.0)
        self.frame_id = str(param(self, 'frame_id', 'odom'))
        self.log_file = str(param(self, 'log_file', ''))
        self.shutdown_on_finish = bool(param(self, 'shutdown_on_finish', False))
        rate = fparam(self, 'control_rate', 50.0)
        odom_topic = param(self, 'odom_topic', '/diff_drive_controller/odom')
        cmd_topic = param(self, 'cmd_vel_topic',
                          '/diff_drive_controller/cmd_vel_unstamped')
        # Prática 04: pose vinda dos sensores; ground truth só para AVALIAR (não entra no controle)
        self.truth_topic = str(param(self, 'truth_topic', ''))
        self.pose_timeout = fparam(self, 'pose_timeout', 0.5)

        self.pose = None
        self.pose_time = None
        self.truth = None
        self.t0 = None
        self.last = None
        self.rmse = RMSE()
        self.rows = []
        self.finished_at = None

        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.des_pub = self.create_publisher(PoseStamped, 'desired_pose', 10)
        self.err_pub = self.create_publisher(Vector3Stamped, 'tracking_error', 10)
        self.rmse_pub = self.create_publisher(Vector3Stamped, 'tracking_rmse', 10)
        self.true_err_pub = self.create_publisher(Vector3Stamped, 'true_tracking_error', 10)
        self.create_subscription(Odometry, odom_topic, self.on_odom, 10)
        if self.truth_topic:
            self.create_subscription(Odometry, self.truth_topic, self.on_truth, 10)
        self.create_timer(1.0 / rate, self.on_timer)
        self.get_logger().info(
            f'Trajetória em 8: modo={self.mode} k_ff={self.k_ff} '
            f'A={traj.A} B={traj.B} omega={traj.omega} T={traj.period:.1f}s '
            f'duração={self.duration:.1f}s')
        self.get_logger().info(
            f'pose do controle: {odom_topic} | ground truth (avaliação): '
            f'{self.truth_topic or "desligado"}')

    def on_odom(self, msg: Odometry):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.pose = Pose2D(p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))
        self.pose_time = self.get_clock().now().nanoseconds * 1e-9

    def on_truth(self, msg: Odometry):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.truth = Pose2D(p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))

    def send(self, v, w):
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = float(v), float(w)
        self.cmd_pub.publish(cmd)

    def on_timer(self):
        if self.pose is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9

        if self.finished_at is None and now - self.pose_time > self.pose_timeout:
            self.get_logger().warn('pose dos sensores desatualizada: parando',
                                   throttle_duration_sec=2.0)
            self.send(0.0, 0.0)
            return

        if self.finished_at is not None:          
            self.send(0.0, 0.0)
            if self.shutdown_on_finish and now - self.finished_at > 0.5:
                rclpy.shutdown()
            return

        if self.t0 is None:
            self.t0 = now + self.start_delay
            self.last = now
            return
        dt, self.last = now - self.last, now
        if dt <= 0.0:
            return
        t = now - self.t0
        if t < 0.0:                               
            self.send(0.0, 0.0)
            return

        v, w, ref, err = self.tracker.compute(t, self.pose, min(dt, 0.2))
        self.send(v, w)
        # erro REAL (ground truth) quando disponível; senão o erro visto pelo controle
        true_err = tracking_error(ref, self.truth) if self.truth is not None else err
        self.rmse.add(true_err)

        stamp = self.get_clock().now().to_msg()
        des = PoseStamped()
        des.header.stamp, des.header.frame_id = stamp, self.frame_id
        des.pose.position.x, des.pose.position.y = ref.x, ref.y
        (des.pose.orientation.x, des.pose.orientation.y,
         des.pose.orientation.z, des.pose.orientation.w) = quaternion_from_yaw(ref.theta)
        self.des_pub.publish(des)

        e = Vector3Stamped()
        e.header.stamp, e.header.frame_id = stamp, self.frame_id
        e.vector.x, e.vector.y, e.vector.z = err.ex_w, err.ey_w, err.eth
        self.err_pub.publish(e)

        te = Vector3Stamped()
        te.header.stamp, te.header.frame_id = stamp, self.frame_id
        te.vector.x, te.vector.y, te.vector.z = true_err.ex_w, true_err.ey_w, true_err.eth
        self.true_err_pub.publish(te)

        r = Vector3Stamped()
        r.header.stamp, r.header.frame_id = stamp, self.frame_id
        r.vector.x, r.vector.z = self.rmse.pos, self.rmse.yaw
        self.rmse_pub.publish(r)

        gt = self.truth if self.truth is not None else Pose2D(math.nan, math.nan, math.nan)
        self.rows.append((t, ref.x, ref.y, ref.theta, self.pose.x, self.pose.y,
                          self.pose.yaw, err.ex_w, err.ey_w, err.eth, v, w,
                          gt.x, gt.y, gt.yaw, true_err.ex_w, true_err.ey_w, true_err.eth))
        if t >= self.duration:
            self.finish()

    def finish(self):
        self.finished_at = self.get_clock().now().nanoseconds * 1e-9
        self.get_logger().info(
            f'FIM [{self.mode}, k_ff={self.k_ff}]: '
            f'RMSE posição{" (ground truth)" if self.truth is not None else ""} = '
            f'{self.rmse.pos:.4f} m, '
            f'RMSE orientação = {self.rmse.yaw:.4f} rad ({math.degrees(self.rmse.yaw):.1f} graus)')
        if self.log_file:
            with open(self.log_file, 'w', newline='') as f:
                wr = csv.writer(f)
                wr.writerow(['t', 'x_d', 'y_d', 'th_d', 'x', 'y', 'th',
                             'ex', 'ey', 'eth', 'v_cmd', 'w_cmd',
                             'x_gt', 'y_gt', 'th_gt', 'ex_gt', 'ey_gt', 'eth_gt'])
                wr.writerows(self.rows)
            self.get_logger().info(f'Log gravado em {self.log_file}')


def main():
    rclpy.init()
    node = TrajectoryFollower()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
