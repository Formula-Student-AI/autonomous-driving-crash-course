import numpy as np
import pytest

from state.vector import State2DVector
from state.vars import yaw, pose


@pytest.fixture
def reference_state():
    return [0.4, 0.6, 0.8, 1.0, 0.0, -0.6, 0.0, 0.0]


class TestStateVector:
    def test_empty_initialisation(self):
        vec = State2DVector()
        assert (vec.to_numpy() == np.asarray([0.0] * 8)).all()

    def test_vector_initialisation(self, reference_state):
        vec = State2DVector(np.asarray(reference_state))
        assert (vec.to_numpy() == np.asarray(reference_state)).all()

    def test_list_initialisation(self, reference_state):
        vec = State2DVector(reference_state)
        assert (vec.to_numpy() == np.asarray(reference_state)).all()

    def test_plus_assignment(self, reference_state):
        vec = State2DVector(reference_state)
        vec += np.asarray([0.1] * 8)
        assert (vec.to_numpy() == np.asarray(reference_state) + 0.1).all()

    def test_minus_assignment(self, reference_state):
        vec = State2DVector(reference_state)
        vec -= np.asarray([0.1] * 8)
        assert (vec.to_numpy() == np.asarray(reference_state) - 0.1).all()

    def test_multiple_assignment_list_mask(self, reference_state):
        vec = State2DVector(reference_state)
        vec[pose] = [-0.4, -0.6, 0.6]

        expected = np.asarray([-0.4, -0.6, 0.6, 1.0, 0.0, -0.6, 0.0, 0.0])
        assert (vec.to_numpy() == expected).all()

    def test_multiple_assignment_numpy_mask(self, reference_state):
        vec = State2DVector(reference_state)
        vec[pose] = np.asarray([-0.4, -0.6, 0.6])

        expected = np.asarray([-0.4, -0.6, 0.6, 1.0, 0.0, -0.6, 0.0, 0.0])
        assert (vec.to_numpy() == expected).all()

    def test_single_assignment_mask(self, reference_state):
        vec = State2DVector(reference_state)
        vec[yaw] = -1.0

        expected = np.asarray([0.4, 0.6, -1.0, 1.0, 0.0, -0.6, 0.0, 0.0])
        assert (vec.to_numpy() == expected).all()

    def test_single_storage(self, reference_state):
        vec = State2DVector(reference_state)
        yaw_value = vec[yaw]
        assert yaw_value == 0.8

    def test_multiple_access(self, reference_state):
        vec = State2DVector(reference_state)
        pose = vec.pose
        assert (pose == [0.4, 0.6, 0.8]).all()

    def test_single_access(self, reference_state):
        vec = State2DVector(reference_state)
        y_position = vec.y
        assert y_position == 0.6

    def test_multiple_assignment_list(self, reference_state):
        vec = State2DVector(reference_state)
        vec.pose = [-0.4, -0.6, 0.6]

        expected = np.asarray([-0.4, -0.6, 0.6, 1.0, 0.0, -0.6, 0.0, 0.0])
        assert (vec.to_numpy() == expected).all()

    def test_multiple_assignment_numpy(self, reference_state):
        vec = State2DVector(reference_state)
        vec.pose = np.asarray([-0.4, -0.6, 0.6])

        expected = np.asarray([-0.4, -0.6, 0.6, 1.0, 0.0, -0.6, 0.0, 0.0])
        assert (vec.to_numpy() == expected).all()

    def test_single_assignment_property(self, reference_state):
        vec = State2DVector(reference_state)
        vec.yaw = -1.0

        expected = np.asarray([0.4, 0.6, -1.0, 1.0, 0.0, -0.6, 0.0, 0.0])
        assert (vec.to_numpy() == expected).all()
