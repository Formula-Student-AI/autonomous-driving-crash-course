#!/usr/bin/env python

from map_lib.type import Cone
from map_lib.fov import WithinSectorChecker, WithinAnnulusChecker

import numpy as np


class TestWithinSectorChecker:
    def test_within_left(self):
        cone = Cone([2., 0.5])
        assert WithinSectorChecker(5.0, 60 * np.pi / 180).inFOV(cone)

    def test_within_right(self):
        cone = Cone([2., -0.5])
        assert WithinSectorChecker(5.0, 60 * np.pi / 180).inFOV(cone)

    def test_outside_left_forward(self):
        cone = Cone([1., 1.])
        assert not WithinSectorChecker(5.0, 60 * np.pi / 180).inFOV(cone)

    def test_outside_right_forward(self):
        cone = Cone([1., -1.])
        assert not WithinSectorChecker(5.0, 60 * np.pi / 180).inFOV(cone)

    def test_outside_left_backward(self):
        cone = Cone([-2., 1.])
        assert not WithinSectorChecker(5.0, 60 * np.pi / 180).inFOV(cone)

    def test_outside_right_backward(self):
        cone = Cone([-2., -1.])
        assert not WithinSectorChecker(5.0, 60 * np.pi / 180).inFOV(cone)


class TestWithinAnnulusChecker:
    def test_within_left(self):
        cone = Cone([-2., -0.5])
        assert WithinAnnulusChecker(5.0, 60 * np.pi / 180, 1.0, np.pi).inFOV(cone)

    def test_within_right(self):
        cone = Cone([-2., 0.5])
        assert WithinAnnulusChecker(5.0, 60 * np.pi / 180, 1.0, np.pi).inFOV(cone)

    def test_outside_left_forward(self):
        cone = Cone([-1., -1.])
        assert not WithinAnnulusChecker(5.0, 60 * np.pi / 180, 1.0, np.pi).inFOV(cone)

    def test_outside_right_forward(self):
        cone = Cone([-1., 1.])
        assert not WithinAnnulusChecker(5.0, 60 * np.pi / 180, 1.0, np.pi).inFOV(cone)

    def test_outside_left_backward(self):
        cone = Cone([2., -1.])
        assert not WithinAnnulusChecker(5.0, 60 * np.pi / 180, 1.0, np.pi).inFOV(cone)

    def test_outside_right_backward(self):
        cone = Cone([2., 1.])
        assert not WithinAnnulusChecker(5.0, 60 * np.pi / 180, 1.0, np.pi).inFOV(cone)

    def test_in_occluded_radius(self):
        cone = Cone([-0.4, 0.1])
        assert not WithinAnnulusChecker(5.0, 60 * np.pi / 180, 1.0, np.pi).inFOV(cone)
