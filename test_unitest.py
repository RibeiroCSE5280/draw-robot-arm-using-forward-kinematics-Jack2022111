import unittest
import importlib
import numpy as np
import pytest
# from importnb import Notebook, get_ipython, imports

from unittest.mock import patch

from robot3D_basic_solution import forward_kinematics


class TestRobotArm(unittest.TestCase):

    def test_forward_kinematics1(self):
        # Lengths of arm parts
        L1 = 5  # Length of link 1
        L2 = 8  # Length of link 2
        L3 = 3  # Length of link 3
        L4 = 0  # Length of link 4
        Phi = np.array([30, -50, -30, 0])
        T_01, T_02, T_03, T_04, e = forward_kinematics(Phi, L1, L2, L3, L4)
        print(e)
        actual = e
        expected = np.array([18.47772028, -0.71432837, 0])

        assert np.allclose(expected, actual)

    def test_forward_kinematics2(self):
        # Lengths of arm parts
        L1 = 5  # Length of link 1
        L2 = 8  # Length of link 2
        L3 = 3  # Length of link 3
        L4 = 0  # Length of link 4
        Phi = np.array([0, 0, 0, 0])
        T_01, T_02, T_03, T_04, e = forward_kinematics(Phi, L1, L2, L3, L4)
        print(e)
        actual = e
        expected = np.array([21, 2, 0])

        assert np.allclose(expected, actual)

    def test_forward_kinematics3(self):
        # Lengths of arm parts
        L1 = 5  # Length of link 1
        L2 = 8  # Length of link 2
        L3 = 3  # Length of link 3
        L4 = 0  # Length of link 4
        Phi = np.array([-30, 50, 30, 0])
        T_01, T_02, T_03, T_04, e = forward_kinematics(Phi, L1, L2, L3, L4)
        print(e)
        actual = e
        expected = np.array([18.47772028, 4.71432837, 0])

        assert np.allclose(expected, actual)


if __name__ == '__main__':
    unittest.main()