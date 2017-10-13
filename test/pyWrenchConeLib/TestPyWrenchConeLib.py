#!/usr/bin/env python

import unittest
import numpy as np
import WrenchConeLib as wcl

# Left foot:
# - pos = array([ 0.2 ,  0.15,  0.1 ])
# - rpy = array([ 0.4, -0. ,  0. ])
# - half-length = 0.11
# - half-width = 0.05
# - friction = 0.5

# Right foot:
# - pos = array([-0.2  , -0.195,  0.   ])
# - rpy = array([-0.4,  0. ,  0. ])
# - half-length = 0.11
# - half-width = 0.05
# - friction = 0.5

# Contact Wrench Cone at [0.0, 0.0, 0.0]:

def compute_cos_sin(angle):
    return np.cos(angle), np.sin(angle)

def rot_matrix_from_rpy(roll, pitch, yaw):
    c, s = compute_cos_sin(roll)
    rot_x = np.array([[1, 0, 0], [0, c, s], [0, -s, c]])
    c, s = compute_cos_sin(pitch)
    rot_y = np.array([[c, 0, -s], [0, 1, 0], [s, 0, c]])
    c, s = compute_cos_sin(yaw)
    rot_z = np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]])
    return rot_x * rot_y * rot_z

class TestPyWrenchConeLib(unittest.TestCase):
    def setUp(self):
        self.com = np.zeros((3,));

        self.lf_pos = np.array([-0.2,  -0.195,  0.1])
        self.lf_rot = rot_matrix_from_rpy(-0.4, 0, 0)
        self.lf_xhl = 0.11
        self.lf_yhl = 0.05
        self.lf_mu = 0.5

        self.rf_pos = np.array([0.2,  0.15,  0.1])
        self.rf_rot = rot_matrix_from_rpy(0.4, 0, 0)
        self.rf_xhl = 0.11
        self.rf_yhl = 0.05
        self.rf_mu = 0.5

    def test_list_input(self):
        a = wcl.ContactSurface([-1., -2., -3.], np.identity(3), [np.zeros((3,))])
        a = wcl.ContactSurface(np.array([-1., -2., -3.]), np.identity(3), [[0, 0, 0], [1, 2, 3]])

    def test_contact_surface(self):
        a = wcl.ContactSurface(np.array([-1., -2., -3.]), np.identity(3), [np.zeros((3,))])
        a = wcl.ContactSurface(np.array([-1., -2., -3.]), np.identity(3), [np.zeros((3,))], 0.5)
        a = wcl.ContactSurface(np.array([-1., -2., -3.]), np.identity(3), [np.zeros((3,))], 0.5, 2)
        a.mu = 0.4
        a.nr_generators = 4
        b = a.position
        a.position = np.array([0., 0., 0.])
        b = a.rotation
        a.rotation = -np.identity(3)
        b = a.points
        a.points = [np.array([1., 0., 0.]), np.array([0., 1., 0.]), np.array([0., 0., 1.])]

    def test_square_contact_surface(self):
        b = wcl.rectangular_surface(0.3, 0.2, np.array([1., 2., 3.]), np.identity(3))
        b = wcl.rectangular_surface(0.3, 0.2, np.array([1., 2., 3.]), np.identity(3), 0.5)
        b = wcl.rectangular_surface(0.3, 0.2, np.array([1., 2., 3.]), np.identity(3), 0.5, 8)

    def test_cwc_1_surf(self):
        a = wcl.ContactSurface(np.array([-1., -2., -3.]), np.identity(3), [np.zeros((3,))])
        cwc = wcl.WrenchCone(self.com, a)

    def test_contact_wrench_cone(self):
        rf_surf = wcl.rectangular_surface(self.rf_xhl, self.rf_yhl, self.rf_pos, self.rf_rot, self.rf_mu)
        lf_surf = wcl.rectangular_surface(self.lf_xhl, self.lf_yhl, self.lf_pos, self.lf_rot, self.lf_mu)
        cwc = wcl.WrenchCone(self.com, [rf_surf, lf_surf])
        cwc_span = cwc.get_rays()
        cwc_face = cwc.get_halfspaces()

    def test_bad_domain_errors(self):
        a = wcl.ContactSurface(np.array([-1., -2., -3.]), np.identity(3), [np.zeros((3,))])
        with self.assertRaises(Exception):
            wcl.WrenchCone(np.array([0, 0, 0]), a) # Vector of int
        with self.assertRaises(Exception):
            wcl.WrenchCone(np.array([0., 0.]), a) # Vector too short
        with self.assertRaises(Exception):
            wcl.WrenchCone(np.array([[0., 0., 0.], [0., 0., 0.]]), a) # Vector bad dimensions
        with self.assertRaises(Exception):
            wcl.WrenchCone(np.array([[0., 0., 0.], [0., 0., 0.]]), [0, 0]) # Bad list of PyContactSurface
        with self.assertRaises(Exception):
            wcl.ContactSurface(np.array([0., 0., 0.]), np.identity(3, dtype=int), [np.zeros((3,))]) # Matrix of int
        with self.assertRaises(Exception):
            wcl.ContactSurface(np.array([0., 0., 0.]), np.identity(2), [np.zeros((3,))]) # Matrix bad dimension
        with self.assertRaises(Exception):
            wcl.ContactSurface(np.array([0., 0., 0.]), np.identity(2), [0]) # Bad list of array

if __name__ == '__main__':
    unittest.main()
