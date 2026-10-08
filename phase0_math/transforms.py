"""
1. 向量是 column vector, 轉換記做 p_a = T_ab @ p_b
2. T_ab : 把 b 座標系的點座標轉換為 a 座標系
3. 右手坐標系、角度為 rad、逆時針為正, 與 ROS REP 103 一致
"""

import numpy as np

def rot2d(theta: float) -> np.ndarray:
    """
    2x2 rotate matrix
    """
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [c, -s],
        [s, c]
    ])

def rot_x(theta: float) -> np.ndarray:
    """
    3x3 rotate matrix around x axis
    """
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [1, 0, 0],
        [0, c, -s],
        [0, s, c]
    ])

def rot_y(theta: float) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [c, 0, s],
        [0, 1, 0],
        [-s, 0, c]
    ])

def rot_z(theta: float) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [c, -s, 0],
        [s, c, 0],
        [0, 0, 1]
    ])

def make_transform(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    """
    由 3x3 rotate matrix 與 3x1 translation vector 組成 4x4 齊次 transform matrix
    """
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = np.asarray(translation).reshape(3)
    return transform

def invert_transform(transform: np.ndarray) -> np.ndarray:
    """
    4x4 齊次 transform matrix 的 inverse matrix
    """
    rotation = transform[:3, :3]
    translation = transform[:3, 3]
    return make_transform(rotation.T, -rotation.T @ translation)