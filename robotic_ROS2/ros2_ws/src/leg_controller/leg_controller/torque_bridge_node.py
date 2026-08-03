import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math

HIP_FORWARD_DEG  = 25.0
HIP_BACKWARD_DEG = -25.0
KNEE_FORWARD_DEG = -25.0
KNEE_BACKWARD_DEG = 0.0
PERIOD_S = 5.0
DT = 0.02

KP_HIP, KD_HIP = 8.0, 0.5
KP_KNEE, KD_KNEE = 6.0, 0.3

def deg2rad(d):
    return d * math.pi / 180.0

class TorqueBridgeNode(Node):
    def __init__(self):
        super().__init__('torque_bridge_node')

        self.traj_pub = self.create_publisher(
            JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)

        self.cmd_pub = self.create_publisher(
            Float64MultiArray, '/leg/torque_cmd', 10)

        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)

        self.feedback_sub = self.create_subscription(
            Float64MultiArray, '/leg/motor_feedback',
            self.motor_feedback_callback, 10)

        self.hip_theta_actual = 0.0
        self.hip_omega_actual = 0.0
        self.knee_theta_actual = 0.0
        self.knee_omega_actual = 0.0

        self.hip_theta_sim = 0.0
        self.hip_omega_sim = 0.0
        self.hip_alpha_sim = 0.0
        self.knee_theta_sim = 0.0
        self.knee_omega_sim = 0.0
        self.knee_alpha_sim = 0.0
        self.hip_theta_hat = 0.0
        self.knee_theta_hat = 0.0

        self.hip_theta_filt = 0.0
        self.knee_theta_filt = 0.0
        self.filter_alpha = 1.0

        self.t0 = self.get_clock().now()
        self.hip_center = (HIP_FORWARD_DEG + HIP_BACKWARD_DEG) / 2.0
        self.hip_amp    = (HIP_FORWARD_DEG - HIP_BACKWARD_DEG) / 2.0
        self.knee_center = (KNEE_FORWARD_DEG + KNEE_BACKWARD_DEG) / 2.0
        self.knee_amp    = (KNEE_FORWARD_DEG - KNEE_BACKWARD_DEG) / 2.0

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

    def motor_feedback_callback(self, msg):
        data = msg.data
        if len(data) < 6:
            self.get_logger().warn(
                f'motor_feedback too short: {len(data)} elements, expected >=6')
            return

        raw_hip_theta = data[0]
        self.hip_omega_sim  = data[1]
        self.hip_alpha_sim  = data[2]
        raw_knee_theta = data[3]
        self.knee_omega_sim = data[4]
        self.knee_alpha_sim = data[5]

        self.hip_theta_sim = raw_hip_theta
        self.knee_theta_sim = raw_knee_theta

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
        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9

        traj_msg = JointTrajectory()
        traj_msg.joint_names = ['hip_joint', 'knee_joint']
        point = JointTrajectoryPoint()
        point.positions = [self.hip_theta_sim, self.knee_theta_sim]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = int(DT * 1e9)
        traj_msg.points = [point]
        self.traj_pub.publish(traj_msg)

        hip_theta, hip_omega, hip_alpha = self.compute_hip(t)
        knee_theta, knee_omega, knee_alpha = self.compute_knee(t)

        hip_theta_des = deg2rad(hip_theta)
        hip_omega_des = deg2rad(hip_omega)
        knee_theta_des = deg2rad(knee_theta)
        knee_omega_des = deg2rad(knee_omega)

        tau_ref_hip = KP_HIP * (hip_theta_des - self.hip_theta_actual) + \
                      KD_HIP * (hip_omega_des - self.hip_omega_actual)
        tau_ref_knee = KP_KNEE * (knee_theta_des - self.knee_theta_actual) + \
                       KD_KNEE * (knee_omega_des - self.knee_omega_actual)

        tau_load_hip = 0.0
        tau_load_knee = 0.0
        cmd_msg = Float64MultiArray()
        cmd_msg.data = [tau_ref_hip, tau_ref_knee, tau_load_hip, tau_load_knee]
        self.cmd_pub.publish(cmd_msg)

        self.get_logger().info(
            f"t={t:.2f} tau_ref_hip={tau_ref_hip:.3f} tau_ref_knee={tau_ref_knee:.3f} "
            f"gazebo_theta_hip={self.hip_theta_actual:.3f} sim_theta_hip={self.hip_theta_sim:.3f} "
            f"ekf_theta_hip={self.hip_theta_hat:.3f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = TorqueBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
