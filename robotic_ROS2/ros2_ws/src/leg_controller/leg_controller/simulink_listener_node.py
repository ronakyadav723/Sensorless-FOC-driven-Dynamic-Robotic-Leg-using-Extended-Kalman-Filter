#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class SimulinkListenerNode(Node):
    def __init__(self):
        super().__init__('simulink_listener_node')
        self.sub = self.create_subscription(
            Float64MultiArray, '/leg/motor_feedback', self.callback, 10)

    def callback(self, msg):
        if len(msg.data) < 6:
            self.get_logger().warn(f"Received incomplete data: {msg.data}")
            return
        theta_hip, omega_hip, alpha_hip, theta_knee, omega_knee, alpha_knee = msg.data
        self.get_logger().info(
            f"hip(theta={theta_hip:.4f}, omega={omega_hip:.4f}, alpha={alpha_hip:.4f}) "
            f"knee(theta={theta_knee:.4f}, omega={omega_knee:.4f}, alpha={alpha_knee:.4f})"
        )

def main(args=None):
    rclpy.init(args=args)
    node = SimulinkListenerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
