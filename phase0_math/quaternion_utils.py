
import math
import numpy as np

def quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """
    Multiply two quaternions.
    """
    x1, y1, z1, w1 = q1[0], q1[1], q1[2], q1[3]
    x2, y2, z2, w2 = q2[0], q2[1], q2[2], q2[3]

    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2

    return np.array([x, y, z, w])

def euler_to_quat(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    Convert Euler angles to a quaternion.
    """
    sin_roll = math.sin(roll/2)
    sin_pitch = math.sin(pitch/2)
    sin_yaw = math.sin(yaw/2)
    cos_roll = math.cos(roll/2)
    cos_pitch = math.cos(pitch/2)
    cos_yaw = math.cos(yaw/2)

    x = sin_roll * cos_pitch * cos_yaw - cos_roll * sin_pitch * sin_yaw
    y = cos_roll * sin_pitch * cos_yaw + sin_roll * cos_pitch * sin_yaw
    z = cos_roll * cos_pitch * sin_yaw - sin_roll * sin_pitch * cos_yaw
    w = cos_roll * cos_pitch * cos_yaw + sin_roll * sin_pitch * sin_yaw

    return np.array([x, y, z, w])

def quat_to_euler(quat: np.ndarray) -> tuple[float, float, float]:
    """
    Convert a quaternion to Euler angles.
    """
    x, y, z, w = quat[0], quat[1], quat[2], quat[3]

    sin_pitch = 2*(w*y - z*x)
    sin_pitch = min(max(sin_pitch, -1.0), 1.0)

    # pitch = ±π/2 時 roll 與 yaw 綁在一起，兩者的 atan2 引數都是 0。
    # 把 yaw 定成 0，剩下的轉角放進 roll。
    if abs(sin_pitch) >= 1.0 - 1e-6:
        sign = math.copysign(1.0, sin_pitch)
        roll = math.atan2(sign*(x*y - z*w), sign*(x*z + y*w))
        pitch = math.copysign(math.pi / 2, sin_pitch)
        yaw = 0.0
    else:
        roll = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
        pitch = math.asin(sin_pitch)
        yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))

    return roll, pitch, yaw