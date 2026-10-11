"""phase0_math/ik.py 的單元測試。"""

import numpy as np
import pytest

from phase0_math.ik import ik
from phase0_math.two_link_arm import forward_kinematics

# 浮點誤差容許值
ATOL = 1e-9


# 兩根連桿刻意取不同長度，l1 與 l2 寫反時才測得出來
@pytest.fixture
def l1() -> float:
    return 1.0


@pytest.fixture
def l2() -> float:
    return 0.5


# 已知關節角，先用 forward_kinematics 算出對應末端座標，再丟進 ik 反解比對，
# 避免自己手算三角函數出錯
@pytest.fixture(
    params=[
        (0.3, 0.8),
        (1.2, -0.8),
        (0.0, np.pi / 2),
        (-0.7, 2.0),
    ],
    ids=[
        "small_positive_angles",
        "mixed_sign_angles",
        "theta1_zero",
        "negative_theta1_large_theta2",
    ],
)
def known_angles(request) -> tuple[float, float]:
    return request.param


@pytest.fixture
def target_xy(known_angles, l1, l2) -> np.ndarray:
    theta1, theta2 = known_angles
    return forward_kinematics(theta1, theta2, l1, l2)


@pytest.fixture
def target_too_far(l1, l2) -> tuple[float, float]:
    # r = l1 + l2 是最大可達距離，取超出它的一點
    max_reach = l1 + l2
    return (max_reach + 0.5, 0.0)


@pytest.fixture
def target_too_close(l1, l2) -> tuple[float, float]:
    # r = |l1 - l2| 是最小可達距離（兩連桿完全摺疊），取小於它的一點
    min_reach = abs(l1 - l2)
    return (min_reach / 2, 0.0)


class TestInverseKinematics:
    def test_both_solutions_round_trip_through_forward_kinematics(self, target_xy, l1, l2):
        x, y = target_xy
        sol_1, sol_2 = ik(x, y, l1, l2)

        fk_1 = forward_kinematics(sol_1[0], sol_1[1], l1, l2)
        fk_2 = forward_kinematics(sol_2[0], sol_2[1], l1, l2)

        np.testing.assert_allclose(fk_1, target_xy, atol=ATOL)
        np.testing.assert_allclose(fk_2, target_xy, atol=ATOL)

    def test_elbow_up_and_elbow_down_have_opposite_theta2_sign(self, target_xy, l1, l2):
        x, y = target_xy
        sol_1, sol_2 = ik(x, y, l1, l2)

        assert sol_1[1] == pytest.approx(-sol_2[1], abs=ATOL)

    def test_target_beyond_max_reach_raises(self, target_too_far, l1, l2):
        x, y = target_too_far

        with pytest.raises(ValueError):
            ik(x, y, l1, l2)

    def test_target_within_min_reach_raises(self, target_too_close, l1, l2):
        x, y = target_too_close

        with pytest.raises(ValueError):
            ik(x, y, l1, l2)
