#!/usr/bin/env python
# -*- coding: utf-8 -*-

import unittest
import numpy as np
import WrenchConeLib as wcl
import timeit

NUMBER_OF_TESTS = 100


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


class System(object):

    def __init__(self):
        self.com = np.zeros((3,))

        self.lf_pos = np.array([-0.2,  -0.195,  0.1])
        self.lf_rot = rot_matrix_from_rpy(-0.4, 0, 0)

        self.rf_pos = np.array([0.2,  0.15,  0.1])
        self.rf_rot = rot_matrix_from_rpy(0.4, 0, 0)

        self.half_length = 0.11
        self.half_width = 0.05
        self.mu = 0.5
        self.nrg = 4

    def compute_rays_perf(self):
        timings = [0.] * NUMBER_OF_TESTS
        for i in xrange(NUMBER_OF_TESTS):
            init_time = timeit.default_timer()

            left_foot_contact = wcl.rectangular_surface(
                self.half_length, self.half_width, self.lf_pos, self.lf_rot, self.mu, self.nrg)
            right_foot_contact = wcl.rectangular_surface(
                self.half_length, self.half_width, self.rf_pos, self.rf_rot, self.mu, self.nrg)

            wrench_cone = wcl.WrenchCone(
                self.com, [left_foot_contact, right_foot_contact])
            rays = wrench_cone.get_rays()

            timings[i] = timeit.default_timer() - init_time

        print "Rays computation time"
        print "Worst of", NUMBER_OF_TESTS, ":", min(timings)*1e6, "µs"
        print "Best of", NUMBER_OF_TESTS, ":", max(timings)*1e6, "µs"
        print "Mean of", NUMBER_OF_TESTS, ":", np.mean(timings)*1e6, "µs"

    def compute_halfspace_perf(self):
        timings = [0.] * NUMBER_OF_TESTS
        for i in xrange(NUMBER_OF_TESTS):
            init_time = timeit.default_timer()

            left_foot_contact = wcl.rectangular_surface(
                self.half_length, self.half_width, self.lf_pos, self.lf_rot, self.mu, self.nrg)
            right_foot_contact = wcl.rectangular_surface(
                self.half_length, self.half_width, self.rf_pos, self.rf_rot, self.mu, self.nrg)

            wrench_cone = wcl.WrenchCone(
                self.com, [left_foot_contact, right_foot_contact])
            rays = wrench_cone.get_halfspaces()

            timings[i] = timeit.default_timer() - init_time

        print
        print "Halfspaces computation time"
        print "Worst of", NUMBER_OF_TESTS, ":", min(timings)*1e6, "µs"
        print "Best of", NUMBER_OF_TESTS, ":", max(timings)*1e6, "µs"
        print "Mean of", NUMBER_OF_TESTS, ":", np.mean(timings)*1e6, "µs"


if __name__ == '__main__':
    system = System()
    system.compute_rays_perf()
    system.compute_halfspace_perf()
