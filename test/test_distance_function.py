"""
A test to check that the distance function returns the correct cost function.
"""
from sst_wrapper.utils.distance_function import TriStateDistance
from sst_wrapper.utils.distance_function import DiffStateDistance
import numpy as np


def test_tri_distance():
    distance_obj = TriStateDistance()
    assert (distance_obj.distance(np.array([0, 0, 0, 0, 0, 0], dtype=np.float),
                                  np.array([1, 1, 1, 1, 1, 1],
                                           dtype=np.float)) == 2)


def test_diff_distance():
    distance_obj = DiffStateDistance()
    assert (distance_obj.distance(
        np.array([0, 0, 0, 0, 0, 0], dtype=np.float),
        np.array([1, 1, 1, 1, 1, 1], dtype=np.float)) == 1.7320508075688772)
