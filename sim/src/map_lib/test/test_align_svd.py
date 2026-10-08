#!/usr/bin/env python
import os

import numpy as np
from scipy.spatial.transform import Rotation

from ament_index_python.packages import get_package_share_directory

from map_lib.type import Map
from map_lib.association.align import AlignSVD

SHARE_PATH = get_package_share_directory("map_lib")
REFERENCE_PATH = os.path.join(SHARE_PATH, "test", "reference_map.csv")


class TestSVD:
    def test_svd(self):
        reference_map = Map.from_csv(REFERENCE_PATH)

        R = np.array(Rotation.from_euler('z', 35, degrees=True).as_matrix()[:2, :2])
        t = np.array([0.2, 0.1])
        transformed_map = Map.from_csv(REFERENCE_PATH).transform(R, t)

        transform = AlignSVD(
            reference_map.positions(),
            transformed_map.positions()).find_transformation()
        assert np.isclose(transform[:2, :2], R).all()
        assert np.isclose(transform[:2, 2], t).all()
