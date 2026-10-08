#!/usr/bin/env python

import os
import filecmp

import numpy as np
import pytest
from ament_index_python.packages import get_package_share_directory

from map_lib.type import Map, Cone, Color

os.environ["QT_API"] = "pyqt6"  # Needed to stop eufs test from failing
SHARE_PATH = get_package_share_directory("map_lib")
REFERENCE_MAP_PATH = os.path.join(SHARE_PATH, "test", "reference_map.csv")
# REFERENCE_MAP_NOISY_PATH = os.path.join(SHARE_PATH, "test", "reference_map_noisy.csv")


def cones_are_approx(val: list[Cone], ref: list[Cone]) -> bool:
    return all([cone_is_approx(cone, ref_cone)] for cone, ref_cone in zip(val, ref))


def cone_is_approx(val: Cone, ref: Cone) -> bool:
    return \
        np.isclose(val.position, ref.position).all() \
        and np.isclose(val.covariance, ref.covariance).all() \
        and val.color == ref.color


@pytest.fixture
def small_reference_map():
    return [
        Cone([0.0, 0.0], [0.4, 0.0, 0.0, 0.2], Color.YELLOW),
        Cone([2.4, 2.0], [0.4, 0.0, 0.0, 0.2], Color.BLUE),
        Cone([0.0, 2.4], [0.4, 0.0, 0.0, 0.2], Color.BLUE),
        Cone([2.2, 0.0], [0.4, 0.0, 0.0, 0.2], Color.YELLOW),
        Cone([1.2, 1.0], [0.4, 0.0, 0.0, 0.2], Color.ORANGE),
        Cone([2.0, 2.2], [0.2, 0.4, 0.4, 0.4], Color.ORANGE)
    ]


@pytest.fixture
def translation_result_map():
    return [
        Cone([-0.2, 0.2], [0.4, 0.0, 0.0, 0.2], Color.YELLOW),
        Cone([2.2, 2.2], [0.4, 0.0, 0.0, 0.2], Color.BLUE),
        Cone([-0.2, 2.6], [0.4, 0.0, 0.0, 0.2], Color.BLUE),
        Cone([2.0, 0.2], [0.4, 0.0, 0.0, 0.2], Color.YELLOW),
        Cone([1.0, 1.2], [0.4, 0.0, 0.0, 0.2], Color.ORANGE),
        Cone([1.8, 2.4], [0.2, 0.4, 0.4, 0.4], Color.ORANGE),
    ]


@pytest.fixture
def inverse_translation_result_map():
    return [
        Cone([-0.2, 0.2], [0.4, 0.0, 0.0, 0.2], Color.YELLOW),
        Cone([2.6, 1.8], [0.4, 0.0, 0.0, 0.2], Color.BLUE),
        Cone([0.2, 2.2], [0.4, 0.0, 0.0, 0.2], Color.BLUE),
        Cone([2.4, -0.2], [0.4, 0.0, 0.0, 0.2], Color.YELLOW),
        Cone([1.4, 0.8], [0.4, 0.0, 0.0, 0.2], Color.ORANGE),
        Cone([2.2, 2.0], [0.2, 0.4, 0.4, 0.4], Color.ORANGE),
    ]


@pytest.fixture
def rotation_result_map():
    return [
        Cone([0.0, 0.0], [0.4, 0.0, 0.0, 0.2], Color.YELLOW),
        Cone([-2.4, -2.0], [0.4, 0.0, 0.0, 0.2], Color.BLUE),
        Cone([0.0, -2.4], [0.4, 0.0, 0.0, 0.2], Color.BLUE),
        Cone([-2.2, 0.0], [0.4, 0.0, 0.0, 0.2], Color.YELLOW),
        Cone([-1.2, -1.0], [0.4, 0.0, 0.0, 0.2], Color.ORANGE),
        Cone([-2.0, -2.2], [0.2, 0.4, 0.4, 0.4], Color.ORANGE),
    ]


@pytest.fixture
def inverse_rotation_result_map():
    return [
        Cone([0.0, 0.0], [0.4, 0.0, 0.0, 0.2], Color.YELLOW),
        Cone([2.0, -2.4], [0.4, 0.0, 0.0, 0.2], Color.BLUE),
        Cone([2.4, 0.0], [0.4, 0.0, 0.0, 0.2], Color.BLUE),
        Cone([0.0, -2.2], [0.4, 0.0, 0.0, 0.2], Color.YELLOW),
        Cone([1.0, -1.2], [0.4, 0.0, 0.0, 0.2], Color.ORANGE),
        Cone([2.2, -2.0], [0.2, 0.4, 0.4, 0.4], Color.ORANGE),
    ]


@pytest.fixture
def transformation_result_map():
    return [
        Cone([-0.2, 0.2], [0.4, 0.0, 0.0, 0.2], Color.YELLOW),
        Cone([-2.6, -1.8], [0.4, 0.0, 0.0, 0.2], Color.BLUE),
        Cone([-0.2, -2.2], [0.4, 0.0, 0.0, 0.2], Color.BLUE),
        Cone([-2.4, 0.2], [0.4, 0.0, 0.0, 0.2], Color.YELLOW),
        Cone([-1.4, -0.8], [0.4, 0.0, 0.0, 0.2], Color.ORANGE),
        Cone([-2.2, -2.0], [0.2, 0.4, 0.4, 0.4], Color.ORANGE),
    ]


@pytest.fixture
def inverse_transformation_result_map():
    return [
        Cone([-0.2, -0.2], [0.4, 0.0, 0.0, 0.2], Color.YELLOW),
        Cone([1.8, -2.6], [0.4, 0.0, 0.0, 0.2], Color.BLUE),
        Cone([2.2, -0.2], [0.4, 0.0, 0.0, 0.2], Color.BLUE),
        Cone([-0.2, -2.4], [0.4, 0.0, 0.0, 0.2], Color.YELLOW),
        Cone([0.8, -1.4], [0.4, 0.0, 0.0, 0.2], Color.ORANGE),
        Cone([2.0, -2.2], [0.2, 0.4, 0.4, 0.4], Color.ORANGE),
    ]


class TestMapConstructor:
    def test_default_constructor(self):
        expected = []
        result = Map()
        assert list(result) == expected

    def test_constructor(self, small_reference_map):
        expected = small_reference_map
        result = Map([
            Cone([0.0, 0.0], [0.4, 0.0, 0.0, 0.2], Color.YELLOW),
            Cone([2.4, 2.0], [0.4, 0.0, 0.0, 0.2], Color.BLUE),
            Cone([0.0, 2.4], [0.4, 0.0, 0.0, 0.2], Color.BLUE),
            Cone([2.2, 0.0], [0.4, 0.0, 0.0, 0.2], Color.YELLOW),
            Cone([1.2, 1.0], [0.4, 0.0, 0.0, 0.2], Color.ORANGE),
            Cone([2.0, 2.2], [0.2, 0.4, 0.4, 0.4], Color.ORANGE)
        ])
        assert list(result) == expected


class TestMapAdd:
    def setup_method(self):
        self.map = Map()

    def test_add_single(self, small_reference_map):
        self.map.add(Cone([0.0, 0.0], [0.4, 0.0, 0.0, 0.2], Color.YELLOW))
        assert list(self.map) == [small_reference_map[0]]

        self.map.add(Cone([2.2, 0.0], [0.4, 0.0, 0.0, 0.2], Color.YELLOW))
        assert list(self.map) == [
            small_reference_map[0],
            small_reference_map[3]
        ]

    def test_add_multiple(self, small_reference_map):
        self.map.add([
            Cone([0.0, 0.0], [0.4, 0.0, 0.0, 0.2], Color.YELLOW),
            Cone([2.4, 2.0], [0.4, 0.0, 0.0, 0.2], Color.BLUE),
            Cone([0.0, 2.4], [0.4, 0.0, 0.0, 0.2], Color.BLUE),
            Cone([2.2, 0.0], [0.4, 0.0, 0.0, 0.2], Color.YELLOW),
            Cone([1.2, 1.0], [0.4, 0.0, 0.0, 0.2], Color.ORANGE),
            Cone([2.0, 2.2], [0.2, 0.4, 0.4, 0.4], Color.ORANGE)
        ])

        assert list(self.map) == small_reference_map


class TestMapDelete:
    def test_delete_one_inplace(self, small_reference_map):
        result = Map.from_csv(REFERENCE_MAP_PATH).delete(1)
        del small_reference_map[1]
        assert list(result) == small_reference_map


class TestMapCones:
    def setup_method(self):
        self.map = Map.from_csv(REFERENCE_MAP_PATH)

    def test_all(self, small_reference_map):
        assert self.map.cones() == small_reference_map

    def test_YELLOW(self, small_reference_map):
        assert self.map.cones(color=Color.YELLOW) == [
            small_reference_map[0],
            small_reference_map[3]
        ]

    def test_indices(self, small_reference_map):
        assert self.map.cones([1, 2, 5]) == [
            small_reference_map[1],
            small_reference_map[2],
            small_reference_map[5]
        ]


class TestMapPositions:
    def test_all(self, small_reference_map):
        result = Map.from_csv(REFERENCE_MAP_PATH).positions()
        assert np.allclose(result, np.array([
            list(map(lambda p: p.position, small_reference_map))
        ]))

    def test_YELLOW(self, small_reference_map):
        result = Map.from_csv(REFERENCE_MAP_PATH).positions(color=Color.YELLOW)
        assert np.allclose(result, np.array([
            small_reference_map[0].position,
            small_reference_map[3].position
        ]))

    def test_indices(self, small_reference_map):
        result = Map.from_csv(REFERENCE_MAP_PATH).positions([1, 2, 5])

        assert np.allclose(result, np.array([
            small_reference_map[1].position,
            small_reference_map[2].position,
            small_reference_map[5].position
        ]))


class TestMapAt:
    def test_at_one_first(self, small_reference_map):
        result = Map.from_csv(REFERENCE_MAP_PATH).at(0)
        assert result == small_reference_map[0]


class TestMapSize:
    def test_zero_size(self):
        assert Map([]).size() == 0

    def test_non_zero_size(self, small_reference_map):
        assert Map(small_reference_map).size() == 6


class TestMapCopy:
    def test_copy(self):
        ref_map = Map.from_csv(REFERENCE_MAP_PATH)
        new_map = ref_map.copy()
        assert ref_map.cones() == new_map.cones()
        assert ref_map is not new_map


class TestMapGetItem:
    def test_at_one_last(self, small_reference_map):
        result = Map.from_csv(REFERENCE_MAP_PATH)[-1]
        assert result == small_reference_map[-1]

    def test_at_range(self, small_reference_map):
        result = Map.from_csv(REFERENCE_MAP_PATH)[1:3]
        assert list(result) == small_reference_map[1:3]


class TestMapSetItem:
    def setup_method(self):
        self.map = Map.from_csv(REFERENCE_MAP_PATH)

    def test_set_single(self, small_reference_map):
        self.map[0] = Cone([6.0, 7.0], [1.4, 0.0, 0.0, 0.2], Color.ORANGE)

        assert self.map.cones() == [
            Cone([6.0, 7.0], [1.4, 0.0, 0.0, 0.2], Color.ORANGE), *small_reference_map[1:]
        ]

    def test_set_range(self, small_reference_map):
        self.map[1:3] = [
            Cone([6.0, 7.0], [1.4, 0.0, 0.0, 0.2], Color.ORANGE),
            Cone([5.0, 4.0], [99.9, 0.0, 0.0, 99.9], Color.YELLOW)
        ]

        assert list(self.map) == [
            small_reference_map[0],
            Cone([6.0, 7.0], [1.4, 0.0, 0.0, 0.2], Color.ORANGE),
            Cone([5.0, 4.0], [99.9, 0.0, 0.0, 99.9], Color.YELLOW),
            *small_reference_map[3:6],
        ]


class TestMapIterator:
    def test_for_loop(self, small_reference_map):
        mutable_map = Map.from_csv(REFERENCE_MAP_PATH)
        for cone in mutable_map:
            cone.position = np.array([7.0, 7.0])

        expected = list(map(lambda c: Cone([7.0, 7.0], c.covariance, c.color),
                            small_reference_map))

        assert list(mutable_map) == expected


class TestMapTranslate:
    def test_no_translate_all_inplace(self):
        t = np.zeros(2)
        ref_map = Map.from_csv(REFERENCE_MAP_PATH)
        result = ref_map.translate(t, inplace=True)
        expected = Map.from_csv(REFERENCE_MAP_PATH)
        assert expected.cones() == result.cones()
        assert ref_map is result

    def test_no_translate_all_copy(self):
        t = np.zeros(2)
        ref_map = Map.from_csv(REFERENCE_MAP_PATH)
        result = ref_map.translate(t)
        expected = Map.from_csv(REFERENCE_MAP_PATH)
        assert expected.cones() == ref_map.cones()
        assert expected.cones() == result.cones()
        assert ref_map is not result

    def test_translate_all_inplace(self, translation_result_map):
        t = np.array([-0.2, 0.2])
        ref_map = Map.from_csv(REFERENCE_MAP_PATH)
        result = ref_map.translate(t, inplace=True)
        assert cones_are_approx(translation_result_map, result.cones())
        assert ref_map is result

    def test_translate_all_copy(self, translation_result_map):
        t = np.array([-0.2, 0.2])
        ref_map = Map.from_csv(REFERENCE_MAP_PATH)
        result = ref_map.translate(t)
        unchanged_map = Map.from_csv(REFERENCE_MAP_PATH)
        assert ref_map.cones() == unchanged_map.cones()
        assert cones_are_approx(translation_result_map, result.cones())
        assert ref_map is not result


class TestMapInverseTranslate:
    def test_inverse_no_translate_all_inplace(self):
        t = np.zeros(2)
        ref_map = Map.from_csv(REFERENCE_MAP_PATH)
        result = ref_map.translate(t, inplace=True, inverse=True)
        expected = Map.from_csv(REFERENCE_MAP_PATH)
        assert expected.cones() == result.cones()
        assert ref_map is result

    def test_inverse_no_translate_all_copy(self):
        t = np.zeros(2)
        ref_map = Map.from_csv(REFERENCE_MAP_PATH)
        result = ref_map.translate(t, inverse=True)
        expected = Map.from_csv(REFERENCE_MAP_PATH)
        assert expected.cones() == ref_map.cones()
        assert expected.cones() == result.cones()
        assert ref_map is not result

    def test_inverse_translate_all_inplace(self, inverse_translation_result_map):
        t = np.array([-0.2, 0.2])
        ref_map = Map.from_csv(REFERENCE_MAP_PATH)
        result = ref_map.translate(t, inplace=True, inverse=True)
        assert cones_are_approx(inverse_translation_result_map, result.cones())
        assert ref_map is result

    def test_inverse_translate_all_copy(self, inverse_translation_result_map):
        t = np.array([-0.2, 0.2])
        ref_map = Map.from_csv(REFERENCE_MAP_PATH)
        result = ref_map.translate(t, inverse=True)
        unchanged_map = Map.from_csv(REFERENCE_MAP_PATH)
        assert ref_map.cones() == unchanged_map.cones()
        assert cones_are_approx(inverse_translation_result_map, result.cones())
        assert ref_map is not result


class TestMapRotateYaw:
    def test_no_rotate_all_inplace(self):
        ref_map = Map.from_csv(REFERENCE_MAP_PATH)
        result = ref_map.rotate_yaw(0, inplace=True)
        expected = Map.from_csv(REFERENCE_MAP_PATH)
        assert expected.cones() == result.cones()
        assert ref_map is result

    def test_no_rotate_all_copy(self):
        ref_map = Map.from_csv(REFERENCE_MAP_PATH)
        result = ref_map.rotate_yaw(0)
        expected = Map.from_csv(REFERENCE_MAP_PATH)
        assert expected.cones() == ref_map.cones()
        assert expected.cones() == result.cones()
        assert ref_map is not result

    def test_rotate_all_inplace(self, rotation_result_map):
        ref_map = Map.from_csv(REFERENCE_MAP_PATH)
        result = ref_map.rotate_yaw(np.pi, inplace=True)
        assert cones_are_approx(rotation_result_map, result.cones())
        assert ref_map is result

    def test_rotate_all_copy(self, rotation_result_map):
        ref_map = Map.from_csv(REFERENCE_MAP_PATH)
        result = ref_map.rotate_yaw(np.pi)
        unchanged_map = Map.from_csv(REFERENCE_MAP_PATH)
        assert ref_map.cones() == unchanged_map.cones()
        assert cones_are_approx(rotation_result_map, result.cones())
        assert ref_map is not result


class TestMapInverseRotateYaw:
    def test_inverse_no_rotate_all_inplace(self):
        ref_map = Map.from_csv(REFERENCE_MAP_PATH)
        result = ref_map.rotate_yaw(0, inplace=True, inverse=True)
        expected = Map.from_csv(REFERENCE_MAP_PATH)
        assert expected.cones() == result.cones()
        assert ref_map is result

    def test_inverse_no_rotate_all_copy(self):
        ref_map = Map.from_csv(REFERENCE_MAP_PATH)
        result = ref_map.rotate_yaw(0, inverse=True)
        expected = Map.from_csv(REFERENCE_MAP_PATH)
        assert expected.cones() == ref_map.cones()
        assert expected.cones() == result.cones()
        assert ref_map is not result

    def test_inverse_rotate_all_inplace(self, inverse_rotation_result_map):
        ref_map = Map.from_csv(REFERENCE_MAP_PATH)
        result = ref_map.rotate_yaw(np.pi, inplace=True, inverse=True)
        assert cones_are_approx(inverse_rotation_result_map, result.cones())
        assert ref_map is result

    def test_inverse_rotate_all_copy(self, inverse_rotation_result_map):
        ref_map = Map.from_csv(REFERENCE_MAP_PATH)
        result = ref_map.rotate_yaw(np.pi, inverse=True)
        unchanged_map = Map.from_csv(REFERENCE_MAP_PATH)
        assert ref_map.cones() == unchanged_map.cones()
        assert cones_are_approx(inverse_rotation_result_map, result.cones())
        assert ref_map is not result


class TestMapTransform:
    def test_no_transform_all_inplace(self):
        R, t = np.eye(2), np.zeros(2)
        ref_map = Map.from_csv(REFERENCE_MAP_PATH)
        result = ref_map.transform(R, t, inplace=True)
        expected = Map.from_csv(REFERENCE_MAP_PATH)
        assert cones_are_approx(expected.cones(), result.cones())
        assert ref_map is result

    def test_no_transform_all_copy(self):
        R, t = np.eye(2), np.zeros(2)
        ref_map = Map.from_csv(REFERENCE_MAP_PATH)
        result = ref_map.transform(R, t)
        expected = Map.from_csv(REFERENCE_MAP_PATH)
        assert expected.cones() == ref_map.cones()
        assert cones_are_approx(expected, result.cones())
        assert ref_map is not result

    def test_transform_all_inplace(self, translation_result_map):
        R, t = np.array([[-1, 0], [0, -1]]), np.array([-0.2, 0.2])
        ref_map = Map.from_csv(REFERENCE_MAP_PATH)
        result = ref_map.transform(R, t, inplace=True)
        assert cones_are_approx(translation_result_map, result.cones())
        assert ref_map is result

    def test_transform_all_copy(self, translation_result_map):
        R, t = np.array([[-1, 0], [0, -1]]), np.array([-0.2, 0.2])
        ref_map = Map.from_csv(REFERENCE_MAP_PATH)
        result = ref_map.transform(R, t)
        unchanged_map = Map.from_csv(REFERENCE_MAP_PATH)
        assert ref_map.cones() == unchanged_map.cones()
        assert cones_are_approx(translation_result_map, result.cones())
        assert ref_map is not result


class TestMapInverseTransform:
    def test_inverse_no_transform_all_inplace(self):
        R, t = np.eye(2), np.zeros(2)
        ref_map = Map.from_csv(REFERENCE_MAP_PATH)
        result = ref_map.transform(R, t, inplace=True, inverse=True)
        expected = Map.from_csv(REFERENCE_MAP_PATH)
        assert cones_are_approx(expected.cones(), result.cones())
        assert ref_map is result

    def test_inverse_no_transform_all_copy(self):
        R, t = np.eye(2), np.zeros(2)
        ref_map = Map.from_csv(REFERENCE_MAP_PATH)
        result = ref_map.transform(R, t, inverse=True)
        expected = Map.from_csv(REFERENCE_MAP_PATH)
        assert expected.cones() == ref_map.cones()
        assert cones_are_approx(expected, result.cones())
        assert ref_map is not result

    def test_inverse_transform_all_inplace(self, inverse_transformation_result_map):
        R, t = np.array([[-1, 0], [0, -1]]), np.array([-0.2, 0.2])
        ref_map = Map.from_csv(REFERENCE_MAP_PATH)
        result = ref_map.transform(R, t, inplace=True, inverse=True)
        assert cones_are_approx(inverse_transformation_result_map, result.cones())
        assert ref_map is result

    def test_inverse_transform_all_copy(self, inverse_transformation_result_map):
        R, t = np.array([[-1, 0], [0, -1]]), np.array([-0.2, 0.2])
        ref_map = Map.from_csv(REFERENCE_MAP_PATH)
        result = ref_map.transform(R, t, inverse=True)
        unchanged_map = Map.from_csv(REFERENCE_MAP_PATH)
        assert ref_map.cones() == unchanged_map.cones()
        assert cones_are_approx(inverse_transformation_result_map, result.cones())
        assert ref_map is not result


class TestMapTransformYaw:
    def test_no_transform_all_inplace(self):
        R, t = 0, np.zeros(2)
        ref_map = Map.from_csv(REFERENCE_MAP_PATH)
        result = ref_map.transform_yaw(R, t, inplace=True)
        expected = Map.from_csv(REFERENCE_MAP_PATH)
        assert expected.cones() == result.cones()
        assert ref_map is result

    def test_no_transform_all_copy(self):
        R, t = 0, np.zeros(2)
        ref_map = Map.from_csv(REFERENCE_MAP_PATH)
        result = ref_map.transform_yaw(R, t)
        expected = Map.from_csv(REFERENCE_MAP_PATH)
        assert expected.cones() == ref_map.cones()
        assert expected.cones() == result.cones()
        assert ref_map is not result

    def test_transform_all_inplace(self, transformation_result_map):
        R, t = np.pi, np.array([-0.2, 0.2])
        ref_map = Map.from_csv(REFERENCE_MAP_PATH)
        result = ref_map.transform_yaw(R, t, inplace=True)
        assert cones_are_approx(transformation_result_map, result.cones())
        assert ref_map is result

    def test_transform_all_copy(self, transformation_result_map):
        R, t = np.pi, np.array([-0.2, 0.2])
        ref_map = Map.from_csv(REFERENCE_MAP_PATH)
        result = ref_map.transform_yaw(R, t)
        unchanged_map = Map.from_csv(REFERENCE_MAP_PATH)
        assert ref_map.cones() == unchanged_map.cones()
        assert cones_are_approx(transformation_result_map, result.cones())
        assert ref_map is not result


class TestMapInverseTransformYaw:
    def test_inverse_no_transform_all_inplace(self):
        R, t = 0, np.zeros(2)
        ref_map = Map.from_csv(REFERENCE_MAP_PATH)
        result = ref_map.transform_yaw(R, t, inplace=True, inverse=True)
        expected = Map.from_csv(REFERENCE_MAP_PATH)
        assert expected.cones() == result.cones()
        assert ref_map is result

    def test_inverse_no_transform_all_copy(self):
        R, t = 0, np.zeros(2)
        ref_map = Map.from_csv(REFERENCE_MAP_PATH)
        result = ref_map.transform_yaw(R, t, inverse=True)
        expected = Map.from_csv(REFERENCE_MAP_PATH)
        assert expected.cones() == ref_map.cones()
        assert expected.cones() == result.cones()
        assert ref_map is not result

    def test_inverse_transform_all_inplace(self, inverse_transformation_result_map):
        R, t = np.pi, np.array([-0.2, 0.2])
        ref_map = Map.from_csv(REFERENCE_MAP_PATH)
        result = ref_map.transform_yaw(R, t, inplace=True, inverse=True)
        assert cones_are_approx(inverse_transformation_result_map, result.cones())
        assert ref_map is result

    def test_inverse_transform_all_copy(self, inverse_transformation_result_map):
        R, t = np.pi, np.array([-0.2, 0.2])
        ref_map = Map.from_csv(REFERENCE_MAP_PATH)
        result = ref_map.transform_yaw(R, t, inverse=True)
        unchanged_map = Map.from_csv(REFERENCE_MAP_PATH)
        assert ref_map.cones() == unchanged_map.cones()
        assert cones_are_approx(inverse_transformation_result_map, result.cones())
        assert ref_map is not result


class TestMapFromCSV:
    def test_invalid_path(self):
        with pytest.raises(FileNotFoundError):
            Map.from_csv(os.path.join(SHARE_PATH, "test", "map_no_exist.csv"))

    def test_empty_file(self):
        result = Map.from_csv(os.path.join(SHARE_PATH, "test", "empty_map.csv"))
        assert result.size() == 0

    def test_small_map(self, small_reference_map):
        expected = small_reference_map
        result = Map.from_csv(REFERENCE_MAP_PATH)
        assert list(result) == expected


class TestMapSave:
    def test_save(self, small_reference_map):
        output_map_path = os.path.join(SHARE_PATH, "map.csv")
        Map(small_reference_map).to_csv(output_map_path)

        # Check that the two files are the same
        assert filecmp.cmp(output_map_path, REFERENCE_MAP_PATH)
