from map_lib.association import SingleAssociation
from map_lib.type import Cone


class TestSingle:
    def test_constructor(self):
        SingleAssociation(1.0)

    def test_single_observation_two_valid_options(self):
        observations = [Cone([1.0, 1.2], [0.1, 0.0, 0.0, 0.1])]
        cone_map = [Cone([0.5, 1.3], [0.4, 0.0, 0.0, 0.4]),
                    Cone([1.0, 1.1], [0.4, 0.0, 0.0, 0.4])]
        association = SingleAssociation(1.0)
        res = association.match(observations, cone_map)
        assert res.matched == {0: 1}
        assert len(res.unmatched) == 0

    def test_single_observation_one_valid_option(self):
        observations = [Cone([1.0, 1.2], [0.1, 0.0, 0.0, 0.1])]
        cone_map = [Cone([0.5, 1.3], [0.4, 0.0, 0.0, 0.4]),
                    Cone([1.0, 1.1], [0.4, 0.0, 0.0, 0.4])]
        association = SingleAssociation(0.25)
        res = association.match(observations, cone_map)
        assert res.matched == {0: 1}
        assert len(res.unmatched) == 0

    def test_single_observation_no_valid_option(self):
        observations = [Cone([1.5, 2.0], [0.1, 0.0, 0.0, 0.1])]
        cone_map = [Cone([0.5, 1.3], [0.4, 0.0, 0.0, 0.4]),
                    Cone([1.0, 1.1], [0.4, 0.0, 0.0, 0.4])]
        association = SingleAssociation(0.25)
        res = association.match(observations, cone_map)
        assert len(res.matched) == 0
        assert res.unmatched == {0}

    def test_unmatched_observations(self):
        observations = [
            Cone([1.0, 1.2], [0.1, 0.0, 0.0, 0.1]),
            Cone([5.9, 1.2], [0.1, 0.0, 0.0, 0.1])
        ]
        cone_map = [
            Cone([0.5, 1.3], [0.4, 0.0, 0.0, 0.4]),
            Cone([1.0, 1.1], [0.4, 0.0, 0.0, 0.4]),
            Cone([5.0, 1.1], [0.4, 0.0, 0.0, 0.4])
        ]
        association = SingleAssociation(0.25)
        res = association.match(observations, cone_map)
        assert res.matched == {0: 1}
        assert res.unmatched == {1}

    def test_duplicate_association(self):
        observations = [
            Cone([1.0, 1.2], [0.1, 0.0, 0.0, 0.1]),
            Cone([0.9, 1.2], [0.1, 0.0, 0.0, 0.1])
        ]
        cone_map = [
            Cone([0.5, 1.3], [0.4, 0.0, 0.0, 0.4]),
            Cone([1.0, 1.1], [0.4, 0.0, 0.0, 0.4]),
            Cone([5.0, 1.1], [0.4, 0.0, 0.0, 0.4])
        ]
        association = SingleAssociation(0.25)
        res = association.match(observations, cone_map)
        assert res.matched == {0: 1, 1: 1}
        assert len(res.unmatched) == 0
