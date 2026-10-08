
import numpy as np

def forward_kinematics(theta1: float, theta2: float, l1: float, l2: float) -> np.ndarray:
    """
    return End Effector Position relative to the base frame
    """
    p1 = np.array([l1 * np.cos(theta1), l1 * np.sin(theta1)])
    p2 = p1 + np.array([l2 * np.cos(theta1 + theta2), l2 * np.sin(theta1 + theta2)])
    return p2