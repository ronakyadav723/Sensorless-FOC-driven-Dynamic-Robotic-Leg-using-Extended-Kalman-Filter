import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import numpy as np
import sys
import csv
import os
from datetime import datetime

sys.path.insert(0, '/home/sharan/sensorless-foc-robotic-leg-fixed/python')
from gait import generate_swing_reference
from motor_params import P, lambda_m

class LegControllerNode(Node):
    def __init__(self):
        super().__init__('leg_controller')

        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10)

        self.torque_pub = self.create_publisher(
            Float64MultiArray, '/leg/torque_command', 10)
        self.angle_pub = self.create_publisher(
            Float64MultiArray, '/leg/joint_angles', 10)

        self.torque_cmd_pub = self.create_publisher(
            Float64MultiArray, '/torque_controller/commands', 10)

        self.ekf_sub = self.create_subscription(
            Float64MultiArray,
            '/motor/ekf_angle',
            self.ekf_callback,
            10)
        self.speed_sub = self.create_subscription(
            Float64MultiArray,
            '/motor/speed_feedback',
            self.speed_callback,
            10)

        self.motor_speed = 0.0
        self.ekf_angle = 0.0
        self.gait_index = 0
        self.tick_count = 0
        self.hip_angles, self.knee_angles, self.gait_time = generate_swing_reference()

        log_dir = os.path.expanduser('~/sensorless-foc-robotic-leg-fixed')
        self.csv_path = os.path.join(log_dir, 'torque_log.csv')
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'timestamp', 'sim_time_s', 'hip_angle_rad', 'knee_angle_rad',
            'ekf_angle_rad', 'iq_ref', 'hip_torque_Nm', 'knee_torque_Nm'
        ])
        self.log_row_count = 0

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info(f'Leg controller node started. Logging torque to {self.csv_path}')

    def ekf_callback(self, msg):
        if msg.data:
            self.ekf_angle = float(msg.data[0])

    def speed_callback(self, msg):
        if msg.data:
            self.motor_speed = float(msg.data[0])

    def control_loop(self):
        idx = self.gait_index % len(self.hip_angles)
        theta1 = self.hip_angles[idx]
        theta2 = self.knee_angles[idx]
        self.gait_index += 5

        traj_msg = JointTrajectory()
        traj_msg.joint_names = ['hip_joint', 'knee_joint']
        point = JointTrajectoryPoint()
        point.positions = [theta1, theta2]
        point.time_from_start = Duration(sec=0, nanosec=100000000)
        traj_msg.points = [point]
        self.trajectory_pub.publish(traj_msg)

        iq_ref = np.clip((theta1 - self.ekf_angle) * 5.0, -10.0, 10.0)
        torque = 1.5 * P * lambda_m * iq_ref
        hip_torque = torque
        knee_torque = torque * 0.6

        torque_msg = Float64MultiArray()
        torque_msg.data = [hip_torque, knee_torque]
        self.torque_pub.publish(torque_msg)
        self.torque_cmd_pub.publish(torque_msg)

        angle_msg = Float64MultiArray()
        angle_msg.data = [theta1, theta2]
        self.angle_pub.publish(angle_msg)

        # Log extra reference points BETWEEN this tick and the previous one, without
        # changing what's actually published to the trajectory/torque controllers.
        # Each control_loop tick advances gait_index by 5, so we log every point the
        # reference passed through in that 5-point span (finer resolution in the CSV
        # than what's actually commanded).
        self.tick_count += 1
        prev_idx = (self.gait_index - 5) % len(self.hip_angles)
        for step in range(200):
            sub_idx = (prev_idx + (step%5)) % len(self.hip_angles)
            sub_hip = self.hip_angles[sub_idx]
            sub_knee = self.knee_angles[sub_idx]
            sub_iq_ref = np.clip((sub_hip - self.ekf_angle) * 5.0, -10.0, 10.0)
            sub_torque = 1.5 * P * lambda_m * sub_iq_ref
            sim_time_s = (self.tick_count - 1) * 0.1 + step * (0.1 / 200.0)
            self.csv_writer.writerow([
                datetime.now().isoformat(),
                f'{sim_time_s:.4f}',
                f'{sub_hip:.6f}',
                f'{sub_knee:.6f}',
                f'{self.ekf_angle:.6f}',
                f'{sub_iq_ref:.6f}',
                f'{sub_torque:.6f}',
                f'{sub_torque * 0.6:.6f}',
            ])
            self.log_row_count += 1

        if self.log_row_count % 100 == 0:
            self.csv_file.flush()

        self.get_logger().info(
            f'Hip: {theta1:.3f} Knee: {theta2:.3f} EKF: {self.ekf_angle:.3f} '
            f'Torque(hip,knee): [{hip_torque:.4f}, {knee_torque:.4f}]')

    def destroy_node(self):
        self.csv_file.flush()
        self.csv_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LegControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
