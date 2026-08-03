import numpy as np
from fk_ik import inverse_kinematics

HIP_OFFSET = np.pi / 2

def generate_swing_reference(cycle_duration=2.0, sample_rate=1000,
                              hip_min=-5*np.pi/36, hip_max=np.pi/6,
                              knee_min=-np.pi/6, knee_max=0.0):
    """
    Simple joint-space reference: hip swings hip_min -> hip_max -> hip_min,
    knee bends knee_max -> knee_min -> knee_max, over one cycle.
    Uses a smooth half-sine so velocity is zero at the endpoints
    (avoids torque spikes at the turn-around points).
    Returns (hip_angles, knee_angles, time_stamps) - same shape as generate_gait().
    """
    num_points = int(cycle_duration * sample_rate)
    dt = cycle_duration / num_points

    hip_angles = []
    knee_angles = []
    time_stamps = []

    for i in range(num_points):
        t = i / num_points  # 0 -> 1 over one full cycle
        phase = np.sin(np.pi * t)  # 0 at t=0, 1 at t=0.5, 0 at t=1

        hip = hip_min + (hip_max - hip_min) * phase
        knee = knee_max + (knee_min - knee_max) * phase  # 0 -> -45deg -> 0

        hip_angles.append(hip)
        knee_angles.append(knee)
        time_stamps.append(i * dt)

    return hip_angles, knee_angles, time_stamps

def generate_gait(cycle_duration=1.0, sample_rate=1000, step_length=0.15,
                   step_height=0.05, stance_y=-0.45):
    num_points = int(cycle_duration * sample_rate)
    half = num_points // 2

    hip_angles = []
    knee_angles = []
    time_stamps = []

    dt = cycle_duration / num_points

    for i in range(half):
        t = i / half
        x = -step_length/2 + step_length * t
        y = stance_y + step_height * np.sin(np.pi * t)
        theta1, theta2 = inverse_kinematics(x, y)
        hip_angles.append(theta1 + HIP_OFFSET)
        knee_angles.append(theta2)
        time_stamps.append(i * dt)

    for i in range(half):
        t = i / half
        x = step_length/2 - step_length * t
        y = stance_y
        theta1, theta2 = inverse_kinematics(x, y)
        hip_angles.append(theta1 + HIP_OFFSET)
        knee_angles.append(theta2)
        time_stamps.append((half + i) * dt)

    return hip_angles, knee_angles, time_stamps


if __name__ == "__main__":
    hip, knee, t = generate_gait()
    print(f"Gait generated successfully")
    print(f"Cycle duration: 1.0 second")
    print(f"Sample rate: 1000 Hz")
    print(f"Number of points: {len(hip)}")
    print(f"Hip angle range: {min(hip):.3f} to {max(hip):.3f} rad")
    print(f"Knee angle range: {min(knee):.3f} to {max(knee):.3f} rad")
    print(f"Time range: {t[0]:.4f}s to {t[-1]:.4f}s")
    print(f"Swing phase points: {len(hip)//2} (semi-elliptical, forward)")
    print(f"Stance phase points: {len(hip)//2} (linear, backward)")

    joint_limit = 1.57
    hip_ok = all(joint_limit >= abs(h) for h in hip)
    knee_ok = all(joint_limit >= abs(k) for k in knee)
    print(f"\nHip within +/-1.57 rad limit: {hip_ok}")
    print(f"Knee within +/-1.57 rad limit: {knee_ok}")
