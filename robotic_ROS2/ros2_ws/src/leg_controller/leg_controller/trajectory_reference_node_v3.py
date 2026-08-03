import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import math
import csv
import time

# --- Set your forward/backward swing limits here (degrees) ---
HIP_FORWARD_DEG  = 35.0
HIP_BACKWARD_DEG = -25.0
KNEE_FORWARD_DEG  = -35.0
KNEE_BACKWARD_DEG = 0.0

PERIOD_S = 15
DT = 0.02
LOG_PATH = "/home/sharan/sensorless-foc-robotic-leg-fixed/ref_trajectory_log_v2.csv"

# PD gains for torque estimate (tune as needed)
KP_HIP, KD_HIP = 15.0, 0.8
KP_KNEE, KD_KNEE = 10.0, 0.5


def deg2rad(d):
    return d * math.pi / 180.0


class TrajectoryReferenceNodeV2(Node):
    def __init__(self):
        super().__init__('trajectory_reference_node_v2')
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # actual state from Gazebo (rad, rad/s)
        self.hip_theta_actual = 0.0
        self.hip_omega_actual = 0.0
        self.knee_theta_actual = 0.0
        self.knee_omega_actual = 0.0

        self.t0 = time.time()

        self.hip_center = (HIP_FORWARD_DEG + HIP_BACKWARD_DEG) / 2.0
        self.hip_amp    = (HIP_FORWARD_DEG - HIP_BACKWARD_DEG) / 2.0
        self.knee_center = (KNEE_FORWARD_DEG + KNEE_BACKWARD_DEG) / 2.0
        self.knee_amp    = (KNEE_FORWARD_DEG - KNEE_BACKWARD_DEG) / 2.0

        self.log_file = open(LOG_PATH, 'w', newline='')
        self.csv_writer = csv.writer(self.log_file)
        self.csv_writer.writerow([
            't',
            'hip_theta_deg', 'hip_omega_dps', 'hip_alpha_dps2', 'hip_torque_nm',
            'knee_theta_deg', 'knee_omega_dps', 'knee_alpha_dps2', 'knee_torque_nm'
        ])
        self.timer = self.create_timer(DT, self.tick)

    def joint_state_callback(self, msg):
        if 'hip_joint' in msg.name:
            i = msg.name.index('hip_joint')
            self.hip_theta_actual = msg.position[i]
            self.hip_omega_actual = msg.velocity[i]
        if 'knee_joint' in msg.name:
            i = msg.name.index('knee_joint')
            self.knee_theta_actual = msg.position[i]
            self.knee_omega_actual = msg.velocity[i]

    def compute_hip(self, t):
        w = 2 * math.pi / PERIOD_S
        theta = self.hip_center + self.hip_amp * math.sin(w * t)
        omega = self.hip_amp * w * math.cos(w * t)
        alpha = -self.hip_amp * (w ** 2) * math.sin(w * t)
        return theta, omega, alpha

    def compute_knee(self, t):
        w = 2 * math.pi / PERIOD_S
        theta = self.knee_center + self.knee_amp * math.sin(w * t)
        omega = self.knee_amp * w * math.cos(w * t)
        alpha = -self.knee_amp * (w ** 2) * math.sin(w * t)
        return theta, omega, alpha

    def tick(self):
        t = time.time() - self.t0
        hip_theta, hip_omega, hip_alpha = self.compute_hip(t)
        knee_theta, knee_omega, knee_alpha = self.compute_knee(t)

        # desired state in radians for torque computation
        hip_theta_des = deg2rad(hip_theta)
        hip_omega_des = deg2rad(hip_omega)
        knee_theta_des = deg2rad(knee_theta)
        knee_omega_des = deg2rad(knee_omega)

        # PD-estimated torque (Nm), using actual state from /joint_states
        hip_torque = KP_HIP * (hip_theta_des - self.hip_theta_actual) + \
                     KD_HIP * (hip_omega_des - self.hip_omega_actual)
        knee_torque = KP_KNEE * (knee_theta_des - self.knee_theta_actual) + \
                      KD_KNEE * (knee_omega_des - self.knee_omega_actual)

        self.csv_writer.writerow([
            f"{t:.4f}",
            f"{hip_theta:.4f}", f"{hip_omega:.4f}", f"{hip_alpha:.4f}", f"{hip_torque:.4f}",
            f"{knee_theta:.4f}", f"{knee_omega:.4f}", f"{knee_alpha:.4f}", f"{knee_torque:.4f}"
        ])
        self.log_file.flush()

        msg = JointTrajectory()
        msg.joint_names = ['hip_joint', 'knee_joint']
        point = JointTrajectoryPoint()
        point.positions = [hip_theta_des, knee_theta_des]
        point.velocities = [hip_omega_des, knee_omega_des]
        point.accelerations = [deg2rad(hip_alpha), deg2rad(knee_alpha)]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = int(DT * 1e9)
        msg.points = [point]
        self.publisher.publish(msg)

        self.get_logger().info(
            f"t={t:.2f}s hip(theta={hip_theta:.1f},omega={hip_omega:.1f},alpha={hip_alpha:.1f},torque={hip_torque:.3f}Nm) "
            f"knee(theta={knee_theta:.1f},omega={knee_omega:.1f},alpha={knee_alpha:.1f},torque={knee_torque:.3f}Nm)"
        )

    def destroy_node(self):
        self.log_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryReferenceNodeV2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

