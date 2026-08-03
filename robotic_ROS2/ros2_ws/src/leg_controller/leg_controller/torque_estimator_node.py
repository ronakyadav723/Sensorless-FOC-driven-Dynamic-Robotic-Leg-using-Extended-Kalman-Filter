#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

# PD gains - tune these based on observed torque magnitude
KP_HIP, KD_HIP = 15.0, 0.8
KP_KNEE, KD_KNEE = 10.0, 0.5

DT = 0.02


class TorqueEstimatorNode(Node):
    def __init__(self):
        super().__init__('torque_estimator_node')

        self.hip_theta_actual = 0.0
        self.hip_omega_actual = 0.0
        self.knee_theta_actual = 0.0
        self.knee_omega_actual = 0.0

        self.hip_theta_des = 0.0
        self.hip_omega_des = 0.0
        self.knee_theta_des = 0.0
        self.knee_omega_des = 0.0

        self.create_subscription(JointState, '/joint_states', self.state_cb, 10)
        self.create_subscription(Float64MultiArray, '/desired_joint_state', self.desired_cb, 10)

        self.torque_pub = self.create_publisher(Float64MultiArray, '/computed_torque', 10)
        self.timer = self.create_timer(DT, self.compute_torque)

    def state_cb(self, msg):
        if 'hip_joint' in msg.name:
            i = msg.name.index('hip_joint')
            self.hip_theta_actual = msg.position[i]
            self.hip_omega_actual = msg.velocity[i]
        if 'knee_joint' in msg.name:
            i = msg.name.index('knee_joint')
            self.knee_theta_actual = msg.position[i]
            self.knee_omega_actual = msg.velocity[i]

    def desired_cb(self, msg):
        # order: [hip_theta, hip_omega, knee_theta, knee_omega] in radians
        self.hip_theta_des = msg.data[0]
        self.hip_omega_des = msg.data[1]
        self.knee_theta_des = msg.data[2]
        self.knee_omega_des = msg.data[3]

    def compute_torque(self):
        hip_torque = KP_HIP * (self.hip_theta_des - self.hip_theta_actual) + \
                     KD_HIP * (self.hip_omega_des - self.hip_omega_actual)
        knee_torque = KP_KNEE * (self.knee_theta_des - self.knee_theta_actual) + \
                      KD_KNEE * (self.knee_omega_des - self.knee_omega_actual)

        msg = Float64MultiArray()
        msg.data = [hip_torque, knee_torque]
        self.torque_pub.publish(msg)

        self.get_logger().info(
            f"computed_torque: hip={hip_torque:.3f} Nm, knee={knee_torque:.3f} Nm"
        )


def main(args=None):
    rclpy.init(args=args)
    node = TorqueEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
