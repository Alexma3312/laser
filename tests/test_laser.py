"""
Unit tests for laser utilities.
"""
from __future__ import print_function

import math
import unittest

import numpy as np

import laser
from sensor_msgs.msg import LaserScan


class TestLaser(unittest.TestCase):
    """Test for laser data process."""

    def setUp(self):
        self.laser_scan = LaserScan()
        self.laser_scan.angle_min = math.radians(-45)
        self.laser_scan.angle_max = math.radians(45)
        self.laser_scan.angle_increment = math.radians(45)
        self.laser_scan.ranges = [math.sqrt(2), 1, math.sqrt(2)]

    def test_points_from_message(self):
        """Test conversion to 2D points."""
        expected = np.transpose(np.array([(1, -1), (1, 0), (1, 1)]))
        actual = laser.points_from_message(self.laser_scan)
        np.testing.assert_array_almost_equal(actual, expected)

    def test_line_from_two_points(self):
        """Test extracting line parameters from two points."""
        p11 = np.array([1, 1])
        pm1 = np.array([-1, 1])
        pmm = np.array([-1, -1])
        p1m = np.array([1, -1])
        self.assertEquals(laser.line_from_two_points(pm1, p11), (1, math.pi/2))
        self.assertEquals(laser.line_from_two_points(p11, pm1), (1, math.pi/2))
        self.assertEquals(laser.line_from_two_points(
            pmm, p11), (0, 3*math.pi/4))
        self.assertEquals(laser.line_from_two_points(pmm, pm1), (-1, 0))
        self.assertEquals(laser.line_from_two_points(
            pmm, p1m), (-1, math.pi/2))
        self.assertEquals(laser.line_from_two_points(pm1, p1m), (0, math.pi/4))
        self.assertEquals(laser.line_from_two_points(p11, p1m), (1, 0))
        self.assertEquals(laser.line_from_two_points(p1m, p11), (1, 0))

    def test_line_distances(self):
        """Test finding distance from line parameterized by range,theta."""
        line = 00, math.pi/2
        points = np.transpose(np.array([(1, -1), (1, 0), (1, 1)]))
        np.testing.assert_array_almost_equal(
            laser.line_distances(line, points), [1, 0, 1])

    def test_find_point(self):
        """Test finding a point in the coodinate with range and angle"""
        best_line1 = 2, math.radians(30)
        best_line2 = -2, math.radians(150)
        best_line3 = -2, math.radians(30)
        best_line4 = -2, math.radians(0)
        best_line5 = 2, math.radians(0)
        best_line6 = -2, math.radians(90)
        np.testing.assert_almost_equal(
            laser.find_point(best_line1), (math.sqrt(3), 1))
        np.testing.assert_almost_equal(
            laser.find_point(best_line2), (math.sqrt(3), -1))
        np.testing.assert_almost_equal(
            laser.find_point(best_line3), (-math.sqrt(3), -1))
        np.testing.assert_almost_equal(laser.find_point(best_line4), (-2, 0))
        np.testing.assert_almost_equal(laser.find_point(best_line5), (2, 0))
        np.testing.assert_almost_equal(laser.find_point(best_line6), (0, -2))

    def test_find_side_distances(self):
        """Test finding left and right distances of a crosspoint to two end points of a line segment"""
        line = (1, 0)
        inliers = [True, True, True]
        np.testing.assert_almost_equal(laser.find_side_distances(
            line, inliers, self.laser_scan), (1, 1))

    def test_find_line(self):
        """Test finding a line in a laser message."""
        rng, angle = 1, 0
        expected = rng, angle
        expected_inliers = [True, True, True]
        actual, actual_inliers = laser.find_line(self.laser_scan)
        np.testing.assert_array_almost_equal(np.array(actual), expected)
        np.testing.assert_array_equal(actual_inliers, expected_inliers)


if __name__ == '__main__':
    unittest.main()
