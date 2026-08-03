import numpy as np

l1 = 0.25
l2 = 0.25

def forward_kinematics(theta1, theta2):
    x = l1 * np.cos(theta1) + l2 * np.cos(theta1 + theta2)
    y = l1 * np.sin(theta1) + l2 * np.sin(theta1 + theta2)
    return x, y

def inverse_kinematics(x, y):
    cos_theta2 = (x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2)
    cos_theta2 = np.clip(cos_theta2, -1, 1)
    theta2 = np.arccos(cos_theta2)
    theta1 = np.arctan2(y, x) - np.arctan2(
        l2 * np.sin(theta2),
        l1 + l2 * np.cos(theta2))
    return theta1, theta2

if __name__ == "__main__":
    theta1_test = 0.5
    theta2_test = 0.8
    x, y = forward_kinematics(theta1_test, theta2_test)
    print(f"FK: ({theta1_test}, {theta2_test}) -> ({x:.4f}, {y:.4f})")
    t1, t2 = inverse_kinematics(x, y)
    print(f"IK: ({x:.4f}, {y:.4f}) -> ({t1:.4f}, {t2:.4f})")
    print(f"Error: ({abs(theta1_test-t1):.6f}, {abs(theta2_test-t2):.6f})")

