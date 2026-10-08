"""phase0_math/two_link_arm.py 的單元測試。"""

import numpy as np
import pytest

from phase0_math.transforms import make_transform, rot_z
from phase0_math.two_link_arm import forward_kinematics

# 浮點誤差容許值；預期值含 0 時相對誤差無法比較，必須給絕對誤差
ATOL = 1e-12


# 兩根連桿刻意取不同長度，l1 與 l2 寫反時才測得出來
@pytest.fixture
def l1() -> float:
    return 1.0


@pytest.fixture
def l2() -> float:
    return 0.5


# 手算結果，對應 l1 = 1.0、l2 = 0.5；theta2 是相對於連桿 1 的角度
@pytest.fixture(
    params=[
        (0.0, 0.0, [1.5, 0.0]),
        (np.pi / 2, 0.0, [0.0, 1.5]),
        (np.pi, 0.0, [-1.5, 0.0]),
        (0.0, np.pi / 2, [1.0, 0.5]),
        (0.0, np.pi, [0.5, 0.0]),
        (np.pi / 2, np.pi / 2, [-0.5, 1.0]),
        (np.pi / 2, -np.pi / 2, [0.5, 1.0]),
    ],
    ids=[
        "both_zero",
        "theta1_quarter_turn",
        "theta1_half_turn",
        "theta2_quarter_turn",
        "theta2_half_turn",
        "both_quarter_turn",
        "theta2_negative_quarter_turn",
    ],
)
def known_pose(request) -> tuple[float, float, list[float]]:
    return request.param


@pytest.fixture
def theta1() -> float:
    return 0.7


@pytest.fixture
def theta2() -> float:
    return -1.1


@pytest.fixture
def other_theta1() -> float:
    return 2.3


@pytest.fixture
def chained_position(theta1, theta2, l1, l2) -> np.ndarray:
    # 末端位置改用齊次轉換鏈算出，不用封閉解，否則是拿自己比自己
    transform_01 = make_transform(rot_z(theta1), np.zeros(3))
    transform_12 = make_transform(rot_z(theta2), np.array([l1, 0.0, 0.0]))
    transform_2e = make_transform(np.eye(3), np.array([l2, 0.0, 0.0]))
    return (transform_01 @ transform_12 @ transform_2e)[:2, 3]


class TestForwardKinematics:
    def test_known_angles_match_hand_calculation(self, known_pose, l1, l2):
        known_theta1, known_theta2, expected = known_pose

        result = forward_kinematics(known_theta1, known_theta2, l1, l2)

        np.testing.assert_allclose(result, expected, atol=ATOL)

    def test_any_angles_return_xy_vector(self, theta1, theta2, l1, l2):
        result = forward_kinematics(theta1, theta2, l1, l2)

        assert result.shape == (2,)

    def test_any_angles_match_chained_transforms(self, theta1, theta2, l1, l2, chained_position):
        result = forward_kinematics(theta1, theta2, l1, l2)

        np.testing.assert_allclose(result, chained_position, atol=ATOL)

    def test_reach_follows_law_of_cosines(self, theta1, theta2, l1, l2):
        result = forward_kinematics(theta1, theta2, l1, l2)

        expected = np.sqrt(l1**2 + l2**2 + 2 * l1 * l2 * np.cos(theta2))
        assert np.linalg.norm(result) == pytest.approx(expected)

    def test_changing_theta1_keeps_reach(self, theta1, other_theta1, theta2, l1, l2):
        # theta1 只會讓整隻手臂繞基座轉，末端到基座的距離不變
        reach = np.linalg.norm(forward_kinematics(theta1, theta2, l1, l2))
        other_reach = np.linalg.norm(forward_kinematics(other_theta1, theta2, l1, l2))

        assert other_reach == pytest.approx(reach)
