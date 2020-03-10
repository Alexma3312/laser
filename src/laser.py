"""
Laser utilities.
"""

import math

import numpy as np

SAMPLE_SIZE = 2  # Number of points required for line fitting


def points_from_message(laser_scan):
    """
    Convert laser ranges to a list of 2D points.

    Keyword arguments:
        laser_scan -- LaserScan ROS message
    Returns:
        a 2*N numpy array
    """
    angles = laser_scan.angle_min + \
        np.arange(len(laser_scan.ranges)) * laser_scan.angle_increment
    return np.vstack((laser_scan.ranges * np.cos(angles), laser_scan.ranges * np.sin(angles)))


def line_from_two_points(point_1, point_2):
    """
    Given two distinct points on a line, return range, angle parameterization of line.

    Keyword arguments:
        point_1 -- first point, a numpy array
        point_2 -- second point, a numpy array
    Returns:
        a range, angle tuple
    Notes:
        angle will always be in [0, pi]
    """

    dx, dy = point_2 - point_1
    # Calculate perpendicular to line
    q = np.array([-dy, dx]) if dx >= 0 else np.array([dy, -dx])
    angle = math.atan2(q[1], q[0])
    rng = np.dot(point_1, q)/np.linalg.norm(q)
    return (-rng, 0) if angle == math.pi else (rng, angle)


def line_distances(line, points):
    """
    Find distance from line parameterized by range,theta to a set of points.

    Keyword arguments:
        line -- rng, theta parameterization of 2D line
        points -- 2*N numpy array
    Returns:
        a 1*N numpy array
    """
    assert points.shape[0] == 2, "expected 2*N array"
    rng, angle = line
    q = np.array([math.cos(angle), math.sin(angle)])
    return np.array([abs(np.dot(point, q) - rng) for point in np.transpose(points)])


def find_point(line):
    """ 
    Use rng and angle to find a point coodinate

    This function is used to find the intersection point of the input line and
    the line that passes through the origin and perpendicular to the input line.

    Keyword arguments:
        rng, angle -- parameterization of a 2d line
    Returns: 
        a coordinate tuple
    """

    rng, angle = line
    return rng*math.cos(angle), rng*math.sin(angle)


# def find_side_distances(line, points, inliers):
def find_side_distances(line, inliers, laser_scan):
    """
    Find left and right distances from a line segment.
    This function can only be used when the x coordinate of the cross_point is positive.

    Keyword arguments:
        left_rng -- the distance of lidar projection on the line to the left end 
        right_rng -- the distance of lidar projection on the line to the right end 
        cross_point -- the intersection point of the input line and 
                        the line that passes through the origin and perpendicular to the input line.
    Returns:
        a 2d tuple (left distance, right distance)
    """

    left_rng = 0
    right_rng = 0
    cross_point = find_point(line)
    points = points_from_message(laser_scan)

    for idx, point in enumerate(np.transpose(points)):
        distance = np.linalg.norm(point - cross_point)
        if inliers[idx]:
            if point[1] > cross_point[1] and distance > right_rng:
                right_rng = distance
            if point[1] < cross_point[1] and distance > left_rng:
                left_rng = distance
    return left_rng, right_rng


def find_line(laser_scan, t=0.1, p=0.95, e=0.3, T=None):
    """
    Use RANSAC to extract line.

    Keyword arguments:
        laser_scan -- LaserScan ROS message
        t -- Distance from the main line
        p -- Chance of hitting a valid pair
        e -- Percentage of outliers
        T -- Early termination threshold 
    Returns:
        line parameters as range, angle, left range, right range tuple
        inliers as a list of bool values
    """

    points = points_from_message(laser_scan)
    dim, num_points = points.shape
    assert dim == 2, "Expected 2D points"

    N = int(math.ceil(math.log((1-p), 10) /
                      math.log((1-math.pow(1-e, SAMPLE_SIZE)), 10)))
    best_line = None
    best_inliers = None
    best_score = 0
    scores = []

    for _iteration in range(N):
        # Sample 2 points
        i, j = np.random.choice(num_points, SAMPLE_SIZE, replace=False)

        # Filter out point pairs that are too close
        if np.linalg.norm(points[:, i]-points[:, j]) < 0.2:
            continue

        # Get line parameters
        line = line_from_two_points(points[:, i], points[:, j])

        # Get inliers
        distances = line_distances(line, points)
        inliers = distances < t
        num_inliers = np.sum(inliers)
        scores.append(num_inliers)

        # If we can, return early
        if T is not None and num_inliers > T:
            return line, inliers

        if best_line is None or num_inliers > best_score:
            best_line = line
            best_inliers = inliers
            best_score = num_inliers

    # if best_score < 100:
    #     print(scores)

    return best_line, best_inliers
