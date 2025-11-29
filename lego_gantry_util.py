"""
Sam Taylor
November 29 2025

This file is a util file, made by copying all of the useful functions
from a companion Google Colab.
"""

import numpy as np


def distance(xyz1, xyz2):
    """just the 3d euclidean distance formula"""
    x1, y1, z1 = xyz1
    x2, y2, z2 = xyz2
    dx = x2 - x1
    dy = y2 - y1
    dz = z2 - z1
    return (dx * dx + dy * dy + dz * dz)**0.5


def calculate_wire_length(tower_position, end_effector_position):
    """Calculate wire length between a tower and the end effector"""
    return distance(end_effector_position, tower_position)


def calculate_wire_length_change(tower_position, current_position, destination_position):
    """Calculate wire length change for a single tower"""
    current_wire_length = calculate_wire_length(tower_position, current_position)
    destination_wire_length = calculate_wire_length(tower_position, destination_position)
    return destination_wire_length - current_wire_length


def calculate_wire_lengths(tower_positions, end_effector_position):
    """Calculate wire lengths between towers and the end effector"""
    wire_lengths = []
    for tower_position in tower_positions:
        wire_lengths.append(distance(end_effector_position, tower_position))
    return wire_lengths


def calculate_wire_length_changes(tower_positions, current_position, destination_position):
    """
    Calculate the length changes of each wire when the end effector goes from
    the current position to a destination position
    """
    current_wire_lengths = calculate_wire_lengths(tower_positions, current_position)
    destination_wire_lengths = calculate_wire_lengths(tower_positions, destination_position)
    wire_length_changes = []
    for current_wire_length, destination_wire_length in zip(current_wire_lengths, destination_wire_lengths):
        wire_length_changes.append(destination_wire_length - current_wire_length)
    return wire_length_changes


def lerp1d(a, b, p):
    """linearly interpolate between a and b, where p is 0-1"""
    return (1 - p) * a + p * b


def lerpnd(v1, v2, p):
    """lerp n-dimensional vectors v1 and v2, where p is 0-1"""
    return [lerp1d(va, vb, p) for va, vb in zip(v1, v2, strict=True)]


def generate_points_on_linear_spline(subsampling, control_points):
    """generate points along a spline, subsampling between each control point"""
    t = np.linspace(0, len(control_points) - 1, (len(control_points) - 1) * subsampling + 1)
    spline_xyz = []
    for i in t[:-1]:
        spline_xyz.append(lerpnd(control_points[int(i)], control_points[int(i) + 1], i % 1))
    # This cuts off the last control point. Add it back:
    spline_xyz.append(control_points[-1])
    return spline_xyz


def generate_points_on_catmull_rom_spline(subsampling, control_points):
    """
    I was stubborn and did this my own way, but there's literally a working
    implementation of a catmull rom spline in python on wikipedia.
    """
    spline_xyz = []
    t = np.linspace(0, len(control_points), len(control_points) * subsampling + 1)
    # catmull-rom characteristic matrix, from the Freya Holmer video
    characteristic_matrix = 0.5 * np.array([
        [ 0,  2,  0,  0],
        [-1,  0,  1,  0],
        [ 2, -5,  4, -1],
        [-1,  3, -3,  1]])
    for i in range(len(control_points)):
        if i == 0 or i >= len(control_points) - 2:
            continue
        for u in t[subsampling * i : subsampling * (i + 1)]:
            polynomial_terms = np.array([1, (u % 1), (u % 1)**2, (u % 1)**3])
            points = np.array(control_points[i - 1 : i + 3])
            curve_point = polynomial_terms @ characteristic_matrix @ points
            spline_xyz.append(curve_point)
    return spline_xyz


def generate_points_on_b_spline(subsampling, control_points):
    """
    Generate points on a b-spline. This doesn't automatically clamp to the
    control points, but you can achieve a clamp by triplicating the first and
    last control points.
    """
    spline_xyz = []
    t = np.linspace(0, len(control_points), len(control_points) * subsampling + 1)
    # b-spline characteristic matrix from Freya Holmer video
    characteristic_matrix = (1 / 6) * np.array([
        [ 1,  4,  1,  0],
        [-3,  0,  3,  0],
        [ 3, -6,  3,  0],
        [-1,  3,  -3, 1]
    ])
    for i in range(len(control_points)):
        if i == 0 or i >= len(control_points) - 2:
            continue
        for u in t[subsampling * i : subsampling * (i + 1)]:
            polynomial_terms = np.array([1, (u % 1), (u % 1)**2, (u % 1)**3])
            points = np.array(control_points[i - 1 : i + 3])
            curve_point = polynomial_terms @ characteristic_matrix @ points
            spline_xyz.append(curve_point)
    return spline_xyz


def approximate_arc_length_of_spline(spline):
    """
    approximate arc length of spline by getting linear distance between
    subsampled spline points
    """
    arc_length = 0
    for i, point in enumerate(spline[:-1]):
        arc_length += distance(point, spline[i + 1])
    return arc_length


def generate_distance_along_spline_lookup_table(spline):
    """
    generate a lookup table such that:
    distance along spline -> point at that distance along the spline
    """
    arc_length = approximate_arc_length_of_spline(spline)
    lookup_table = {0: spline[0]}
    distance_so_far = 0
    for i, point in enumerate(spline[:-1]):
        distance_so_far += distance(point, spline[i + 1])
        lookup_table.update({(distance_so_far / arc_length): spline[i + 1]})
    return lookup_table


def get_spline_point_from_distance_along_spline(lookup_table, distance_along_spline):
    """
    get spline from a lookup table where
    distance along spline -> point at that distance along the spline
    and interpolate if the key distance isn't in the lookup table
    """
    # look up the point in the lookup table and return it
    point = lookup_table.get(distance_along_spline, None)
    if point is not None:
        return point
    # interpolate between existing points in the lookup table
    closest_key_below = 0
    closest_key_above = 1
    # dictionaries are unsorted so I can't use a binary search. Check every key.
    for key in lookup_table.keys():
        if (key < distance_along_spline
            and (distance_along_spline - key) < (distance_along_spline - closest_key_below)):
            closest_key_below = key
        if (key > distance_along_spline
            and (key - distance_along_spline) < (closest_key_above - distance_along_spline)):
            closest_key_above = key
    return lerpnd(lookup_table[closest_key_below],
                  lookup_table[closest_key_above],
                  (distance_along_spline - closest_key_below) / (closest_key_above - closest_key_below))


def generate_velocities_on_b_spline(subsampling, control_points):
    """
    generate velocity vectors on a b-spline. Clamp to the end control points.
    """
    spline_xyz = []
    t = np.linspace(0, len(control_points), len(control_points) * subsampling + 1)
    # b-spline characteristic matrix from Freya Holmer video
    characteristic_matrix = (1 / 6) * np.array([
        [ 1,  4,  1,  0],
        [-3,  0,  3,  0],
        [ 3, -6,  3,  0],
        [-1,  3,  -3, 1]
    ])
    for i in range(len(control_points)):
        if i == 0 or i >= len(control_points) - 2:
            continue
        for u in t[subsampling * i : subsampling * (i + 1)]:
            polynomial_terms = np.array([0, 1, 2 * (u % 1), 3 * (u % 1)**2])
            points = np.array(control_points[i - 1 : i + 3])
            curve_point = polynomial_terms @ characteristic_matrix @ points
            spline_xyz.append(curve_point)
    return spline_xyz


def generate_wire_velocities(vt, pt, tower_position):
    # assume the same subsampling as vt, pt
    wire_velocities = []
    for end_effector_velocity, end_effector_position in zip(vt, pt):
        wire_vector = end_effector_position - tower_position
        wire_vector_magn = np.linalg.norm(wire_vector)
        wire_direction = wire_vector / wire_vector_magn
        velocity_along_wire_3d = np.dot(np.dot(end_effector_velocity, wire_direction), wire_direction)
        speed_along_wire = np.linalg.norm(velocity_along_wire_3d)
        direction_along_wire = 1 if np.dot(velocity_along_wire_3d, wire_direction) > 0 else -1
        velocity_along_wire_1d = speed_along_wire * direction_along_wire
        wire_velocities.append(velocity_along_wire_1d)
    return wire_velocities