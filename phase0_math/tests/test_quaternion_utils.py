"""phase0_math/quaternion_utils.py 的單元測試。

慣例：四元數順序 (x, y, z, w)；尤拉角為 REP 103 固定軸 roll-pitch-yaw，
即 R = Rz(yaw) @ Ry(pitch) @ Rx(roll)，對應 scipy 小寫的 'xyz'。
"""

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from phase0_math.quaternion_utils import euler_to_quat, quat_multiply, quat_to_euler
from phase0_math.transforms import rot_x, rot_y, rot_z

# 浮點誤差容許值
ATOL = 1e-9


def assert_same_rotation(q_actual, q_expected):
    # q 與 -q 是同一個旋轉，所以比對單位四元數內積的絕對值是否為 1
    dot = abs(np.dot(q_actual, q_expected))
    np.testing.assert_allclose(dot, 1.0, atol=ATOL)


# 三個角度刻意取互不相同的值，軸順序寫錯時才測得出來；pitch 都在 (-pi/2, pi/2) 內
@pytest.fixture(
    params=[
        (0.3, 0.5, 0.7),
        (-1.0, 0.4, 2.0),
        (2.5, -0.9, -1.3),
        (-2.8, -1.2, 0.2),
        (0.0, 0.0, 0.0),
    ],
    ids=[
        "all_positive",
        "negative_roll",
        "negative_pitch_and_yaw",
        "large_negative_roll",
        "identity",
    ],
)
def general_angles(request) -> tuple[float, float, float]:
    return request.param


@pytest.fixture
def scipy_rotation(general_angles) -> Rotation:
    return Rotation.from_euler("xyz", list(general_angles))


# 萬向鎖：pitch = +-pi/2，搭配非零的 roll 與 yaw
@pytest.fixture(
    params=[
        (0.4, np.pi / 2, 0.9),
        (-1.1, -np.pi / 2, 0.6),
    ],
    ids=["pitch_plus_half_pi", "pitch_minus_half_pi"],
)
def gimbal_angles(request) -> tuple[float, float, float]:
    return request.param


# 兩個旋轉取不同軸（x 軸與 y 軸），同軸旋轉可交換，順序寫反時測不出來
@pytest.fixture
def rotation_about_x() -> Rotation:
    return Rotation.from_rotvec([0.8, 0.0, 0.0])


@pytest.fixture
def rotation_about_y() -> Rotation:
    return Rotation.from_rotvec([0.0, 1.1, 0.0])


@pytest.fixture
def identity_quat() -> np.ndarray:
    return np.array([0.0, 0.0, 0.0, 1.0])


class TestEulerToQuat:
    def test_general_angles_match_scipy(self, general_angles, scipy_rotation):
        roll, pitch, yaw = general_angles

        result = euler_to_quat(roll, pitch, yaw)

        assert_same_rotation(result, scipy_rotation.as_quat())

    def test_general_angles_return_unit_quaternion(self, general_angles):
        roll, pitch, yaw = general_angles

        result = euler_to_quat(roll, pitch, yaw)

        np.testing.assert_allclose(np.linalg.norm(result), 1.0, atol=ATOL)

    def test_general_angles_match_rotation_matrix_from_transforms(self, general_angles):
        roll, pitch, yaw = general_angles
        expected = rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)

        result = Rotation.from_quat(euler_to_quat(roll, pitch, yaw)).as_matrix()

        np.testing.assert_allclose(result, expected, atol=ATOL)


class TestQuatToEuler:
    def test_general_angles_match_scipy(self, general_angles, scipy_rotation):
        result = quat_to_euler(scipy_rotation.as_quat())

        np.testing.assert_allclose(result, scipy_rotation.as_euler("xyz"), atol=ATOL)

    def test_round_trip_through_euler_to_quat_recovers_angles(self, general_angles):
        roll, pitch, yaw = general_angles

        result = quat_to_euler(euler_to_quat(roll, pitch, yaw))

        np.testing.assert_allclose(result, general_angles, atol=ATOL)

    def test_gimbal_lock_result_represents_same_rotation(self, gimbal_angles):
        # 萬向鎖時尤拉角不唯一，改比轉回四元數後是否為同一個旋轉
        original = euler_to_quat(*gimbal_angles)

        recovered = euler_to_quat(*quat_to_euler(original))

        assert_same_rotation(recovered, original)


class TestQuatMultiply:
    def test_different_axes_match_scipy(self, rotation_about_x, rotation_about_y):
        expected = (rotation_about_x * rotation_about_y).as_quat()

        result = quat_multiply(rotation_about_x.as_quat(), rotation_about_y.as_quat())

        assert_same_rotation(result, expected)

    def test_swapped_order_gives_different_rotation(self, rotation_about_x, rotation_about_y):
        q_x = rotation_about_x.as_quat()
        q_y = rotation_about_y.as_quat()

        forward = quat_multiply(q_x, q_y)
        swapped = quat_multiply(q_y, q_x)

        assert abs(np.dot(forward, swapped)) < 1.0 - 1e-3

    def test_swapped_order_matches_scipy_swapped_product(self, rotation_about_x, rotation_about_y):
        expected = (rotation_about_y * rotation_about_x).as_quat()

        result = quat_multiply(rotation_about_y.as_quat(), rotation_about_x.as_quat())

        assert_same_rotation(result, expected)

    def test_identity_on_left_returns_other_quaternion(self, identity_quat, rotation_about_y):
        q = rotation_about_y.as_quat()

        result = quat_multiply(identity_quat, q)

        np.testing.assert_allclose(result, q, atol=ATOL)

    def test_identity_on_right_returns_other_quaternion(self, identity_quat, rotation_about_y):
        q = rotation_about_y.as_quat()

        result = quat_multiply(q, identity_quat)

        np.testing.assert_allclose(result, q, atol=ATOL)

    def test_product_of_unit_quaternions_is_unit_length(self, rotation_about_x, rotation_about_y):
        result = quat_multiply(rotation_about_x.as_quat(), rotation_about_y.as_quat())

        np.testing.assert_allclose(np.linalg.norm(result), 1.0, atol=ATOL)
