"""phase0_math/transforms.py 的單元測試。"""

import numpy as np
import pytest

from phase0_math.transforms import (
    invert_transform,
    make_transform,
    rot2d,
    rot_x,
    rot_y,
    rot_z,
)

# 浮點誤差容許值；預期值含 0 時相對誤差無法比較，必須給絕對誤差
ATOL = 1e-12


@pytest.fixture
def theta() -> float:
    return 0.7


@pytest.fixture
def quarter_turn() -> float:
    return np.pi / 2


@pytest.fixture(params=[rot_x, rot_y, rot_z], ids=["rot_x", "rot_y", "rot_z"])
def rotation_3d(request, theta) -> np.ndarray:
    return request.param(theta)


@pytest.fixture
def x_axis() -> np.ndarray:
    return np.array([1.0, 0.0, 0.0])


@pytest.fixture
def y_axis() -> np.ndarray:
    return np.array([0.0, 1.0, 0.0])


@pytest.fixture
def z_axis() -> np.ndarray:
    return np.array([0.0, 0.0, 1.0])


@pytest.fixture
def rotation_ab() -> np.ndarray:
    return rot_z(0.3) @ rot_x(-1.1)


@pytest.fixture
def translation_ab() -> np.ndarray:
    return np.array([1.0, -2.0, 0.5])


@pytest.fixture
def rotation_bc() -> np.ndarray:
    return rot_y(0.8) @ rot_z(2.0)


@pytest.fixture
def translation_bc() -> np.ndarray:
    return np.array([-0.4, 3.0, 1.5])


@pytest.fixture
def transform_ab(rotation_ab, translation_ab) -> np.ndarray:
    return make_transform(rotation_ab, translation_ab)


@pytest.fixture
def transform_bc(rotation_bc, translation_bc) -> np.ndarray:
    return make_transform(rotation_bc, translation_bc)


@pytest.fixture
def point_c() -> np.ndarray:
    return np.array([0.2, -0.7, 1.3])


class TestRot2d:
    def test_zero_angle_returns_identity(self):
        np.testing.assert_allclose(rot2d(0.0), np.eye(2), atol=ATOL)

    def test_quarter_turn_maps_x_axis_to_y_axis(self, quarter_turn):
        result = rot2d(quarter_turn) @ np.array([1.0, 0.0])

        np.testing.assert_allclose(result, [0.0, 1.0], atol=ATOL)

    def test_any_angle_is_orthonormal_with_unit_determinant(self, theta):
        rotation = rot2d(theta)

        np.testing.assert_allclose(rotation @ rotation.T, np.eye(2), atol=ATOL)
        assert np.linalg.det(rotation) == pytest.approx(1.0)


class TestRot3d:
    def test_any_axis_is_orthonormal_with_unit_determinant(self, rotation_3d):
        np.testing.assert_allclose(rotation_3d @ rotation_3d.T, np.eye(3), atol=ATOL)
        assert np.linalg.det(rotation_3d) == pytest.approx(1.0)

    # 以下三個測試固定右手系、逆時針為正的方向；負號放錯位置會在這裡被抓到
    def test_rot_x_quarter_turn_maps_y_axis_to_z_axis(self, quarter_turn, y_axis, z_axis):
        np.testing.assert_allclose(rot_x(quarter_turn) @ y_axis, z_axis, atol=ATOL)

    def test_rot_y_quarter_turn_maps_z_axis_to_x_axis(self, quarter_turn, z_axis, x_axis):
        np.testing.assert_allclose(rot_y(quarter_turn) @ z_axis, x_axis, atol=ATOL)

    def test_rot_z_quarter_turn_maps_x_axis_to_y_axis(self, quarter_turn, x_axis, y_axis):
        np.testing.assert_allclose(rot_z(quarter_turn) @ x_axis, y_axis, atol=ATOL)

    def test_rot_z_upper_left_block_equals_rot2d(self, theta):
        np.testing.assert_allclose(rot_z(theta)[:2, :2], rot2d(theta), atol=ATOL)


class TestMakeTransform:
    def test_rotation_and_translation_fill_expected_blocks(
        self, transform_ab, rotation_ab, translation_ab
    ):
        assert transform_ab.shape == (4, 4)
        np.testing.assert_allclose(transform_ab[:3, :3], rotation_ab, atol=ATOL)
        np.testing.assert_allclose(transform_ab[:3, 3], translation_ab, atol=ATOL)

    def test_bottom_row_is_homogeneous(self, transform_ab):
        np.testing.assert_allclose(transform_ab[3, :], [0.0, 0.0, 0.0, 1.0], atol=ATOL)

    def test_column_vector_translation_matches_flat_translation(
        self, transform_ab, rotation_ab, translation_ab
    ):
        result = make_transform(rotation_ab, translation_ab.reshape(3, 1))

        np.testing.assert_allclose(result, transform_ab, atol=ATOL)

    def test_homogeneous_point_is_rotated_then_translated(
        self, transform_ab, rotation_ab, translation_ab, point_c
    ):
        result = transform_ab @ np.append(point_c, 1.0)

        np.testing.assert_allclose(result[:3], rotation_ab @ point_c + translation_ab, atol=ATOL)
        assert result[3] == pytest.approx(1.0)


class TestInvertTransform:
    def test_transform_times_inverse_returns_identity(self, transform_ab):
        result = transform_ab @ invert_transform(transform_ab)

        np.testing.assert_allclose(result, np.eye(4), atol=ATOL)

    def test_inverse_times_transform_returns_identity(self, transform_ab):
        result = invert_transform(transform_ab) @ transform_ab

        np.testing.assert_allclose(result, np.eye(4), atol=ATOL)

    def test_closed_form_matches_numpy_inverse(self, transform_ab):
        np.testing.assert_allclose(
            invert_transform(transform_ab), np.linalg.inv(transform_ab), atol=ATOL
        )


class TestComposition:
    def test_chained_transforms_equal_directly_built_transform(
        self, transform_ab, transform_bc, rotation_ab, translation_ab, rotation_bc, translation_bc
    ):
        # T_ac 由合成後的旋轉與平移另外建出，不用 T_ab @ T_bc 算，否則是拿自己比自己
        transform_ac = make_transform(
            rotation_ab @ rotation_bc, rotation_ab @ translation_bc + translation_ab
        )

        np.testing.assert_allclose(transform_ab @ transform_bc, transform_ac, atol=ATOL)

    def test_chained_transform_matches_step_by_step_point_conversion(
        self, transform_ab, transform_bc, rotation_ab, translation_ab, rotation_bc, translation_bc, point_c
    ):
        point_b = rotation_bc @ point_c + translation_bc
        point_a = rotation_ab @ point_b + translation_ab

        result = transform_ab @ transform_bc @ np.append(point_c, 1.0)

        np.testing.assert_allclose(result[:3], point_a, atol=ATOL)
