#!/usr/bin/env python3
"""TriFingerPro Inverse Kinematics.

Direct, analytic inverse kinematics solution for the TriFingerPro, see
:func:`trifingerpro_ik`.

Below is the description of the math used in the functions.


Solution for finger0
====================

.. todo:: add drawing of finger with x/y/z axes

Upper joint q0
--------------

.. image:: doc/images/trifingerpro_ik_drawing_q0.png

The upper joint is rotating around the y-axis, so for finding its angle, it is enough to
look at the xz-plane.  In zero-configuration, the tip is offset from the base in
x-direction by TIP_X_OFFSET.  So for finding the angular joint position ``q0``, we draw
a circle with radius TIP_X_OFFSET around the base center (which is the point of
rotation) and find the tangent to that circle that goes through the finger tip.  The
angle between this tangent and the z-axis directly gives the joint angle ``q0``.

Note that there are two tangents, so we have to determine which one is the right one.
This is done by checking the angle between the tip-to-base and tip-to-tangent-point
vectors.  For the correct tangent, this angle has to be negative.

The other two joints only affect the position of the finger tip on the tangent line, so
``q0`` is independent of them.


Middle joint q1
---------------

.. image:: doc/images/trifingerpro_ik_drawing_q1_and_q2.png

To compute q1, we project the tip position, middle joint and lower joint onto the
yz-plane rotated by q0 (basically flattening the robot along the x-axis in
zero-configuration).  This plane is referred to as "tangent plane" in the code, because
plane is orthogonal to the xz-plane and parallel with the tangent computed above.

On this plane, we can compute the distance between the finger tip and the middle joint
(joint 1).  Together with the known lengths of the middle and lower link, we get a
triangle where the lengths of all three sides is known.  We can easily compute the
angles ``alpha + q1`` and ``beta`` of it.

Now we only need to find ``alpha`` to know ``q1``.  We get it by rotating the
tip-to-joint1 vector from the tangent plane by ``-q0`` to align with the xz-plane and
then compute the angle between z-axis and the rotated tip-to-joint1 vector.


Lower joint q2
--------------

Using the triangle from above, we can easily compute ``beta``.  Then ``q2 = beta -
180°``.


Solution for finger120 and finger240
====================================

Since the other fingers are simply rotated around the center by 120° and 240° with
respect to finger0, we can rotate the desired tip positions for these fingers in the
opposite direction and then simply use the same function as for finger0 to compute the
joint angles.

"""

import numpy as np

# from the URDF model
BASE_HEIGHT = 0.29
TIP_X_OFFSET = 0.086
MIDDLE_LINK_LENGTH = 0.16
LOWER_LINK_LENGTH = 0.16
JOINT1_Y_OFFSET = 0.0505


def _get_tangent_points(
    px: float, py: float, cx: float, cy: float, radius: float
) -> tuple[np.ndarray, np.ndarray] | None:
    """Compute tangents of a circle that go through a given point P outside the circle.

    There are two solutions, so the function is returning two tangent points.

    Args:
        px: x-coordinate of the point P.
        py: y-coordinate of the point P.
        cx: x-coordinate of the circle center
        cy: y-coordinate of the circle center
        radius: radius of the circle

    Returns:
        The tangent points on the circle of the two tangents.  Returns None if P lies
        inside the circle.
    """
    # based on https://stackoverflow.com/a/69641745/2095383
    dx = cx - px
    dy = cy - py
    if dx == 0 and dy == 0:
        return None

    # PC is distance between P and C, pc2 is PC^2
    pc2 = dx * dx + dy * dy
    pc = np.sqrt(pc2)
    if pc < radius:
        return None

    # R is radius of  circle centered in P, r2 is R^2
    r2 = pc2 - radius * radius

    # d is the P => X0 distance (demonstration is here
    # https://mathworld.wolfram.com/Circle-CircleIntersection.html where PC is named 'd'
    # in there)
    d = r2 / pc

    # h is the X0 => X1 (and X0 => X2) distance
    h = np.sqrt(r2 - d * d)

    # first tangent point
    point1 = np.array(
        [
            px + (dx * d - dy * h) / pc,
            py + (dy * d + dx * h) / pc,
        ]
    )

    # second tangent point
    point2 = np.array(
        [
            px + (dx * d + dy * h) / pc,
            py + (dy * d - dx * h) / pc,
        ]
    )

    return point1, point2


def fingerpro_ik(tip_position: np.ndarray) -> np.ndarray:
    """Compute inverse kinematics for a single FingerPro.

    Args:
        tip_position: The desired tip position in world coordinates.

    Returns:
        Joint angles for the finger to reach the given tip position.
    """
    tip_position = np.asarray(tip_position)

    # JOINT 0 (upper joint) ==================================
    tip_xz = tip_position[[0, 2]]

    tangent_points = _get_tangent_points(
        tip_xz[0], tip_xz[1], 0, BASE_HEIGHT, TIP_X_OFFSET
    )
    if tangent_points is None:
        raise ValueError("No tangent points found, tip is too close to the base")

    # We get two tangent points from the function above.  The correct one is the one
    # where the angle between tip-to-base and tip-to-tangent is negative.
    tip_to_base = np.array([0, BASE_HEIGHT]) - tip_xz
    tip_to_tangent0 = tangent_points[0] - tip_xz
    # https://wumbo.net/formulas/angle-between-two-vectors-2d/
    # angle from v to w: atan2(v1w2-v2w1, v1w1+v2w2)
    angle_base_to_tangent = np.arctan2(
        tip_to_base[0] * tip_to_tangent0[1] - tip_to_base[1] * tip_to_tangent0[0],
        tip_to_base[0] * tip_to_tangent0[0] + tip_to_base[1] * tip_to_tangent0[1],
    )
    # we are looking for the tangent point where the angle is negative
    if angle_base_to_tangent < 0:
        tangent_point = tangent_points[0]
    else:
        tangent_point = tangent_points[1]

    tangent_vector = tangent_point - tip_xz
    tangent_vector /= np.linalg.norm(tangent_vector)
    joint0_angle = np.pi / 2 - np.arctan2(tangent_vector[1], tangent_vector[0])

    # JOINT 1 (middle joint) ==================================

    rot_y = np.array(
        [
            [np.cos(joint0_angle), 0.0, np.sin(joint0_angle)],
            [0.0, 1.0, 0.0],
            [-np.sin(joint0_angle), 0.0, np.cos(joint0_angle)],
        ]
    )

    rot_y_inv = np.array(
        [
            [np.cos(-joint0_angle), 0.0, np.sin(-joint0_angle)],
            [0.0, 1.0, 0.0],
            [-np.sin(-joint0_angle), 0.0, np.cos(-joint0_angle)],
        ]
    )

    # project the joint 1 position to the tangent plane
    joint1_pos_yz = np.array([0, JOINT1_Y_OFFSET, BASE_HEIGHT])
    vec_joint1_pos_yz_to_tip_plane = np.array([TIP_X_OFFSET, 0, 0])
    joint1_on_tip_plane = joint1_pos_yz + (rot_y @ vec_joint1_pos_yz_to_tip_plane)

    tip_to_joint1 = joint1_on_tip_plane - tip_position
    tip_to_joint1_distance = np.linalg.norm(tip_to_joint1)

    # rotate tip-to_joint_on_plane to align with z-azis
    tip_to_joint1_foo = rot_y_inv @ tip_to_joint1
    # normalize for angle calculation
    tip_to_joint1_foo /= np.linalg.norm(tip_to_joint1_foo)
    alpha = np.pi / 2 - np.arctan2(tip_to_joint1_foo[2], tip_to_joint1_foo[1])

    alpha_and_q1 = np.arccos(
        (tip_to_joint1_distance**2 + MIDDLE_LINK_LENGTH**2 - LOWER_LINK_LENGTH**2)
        / (2 * tip_to_joint1_distance * MIDDLE_LINK_LENGTH)
    )
    joint1_angle = alpha_and_q1 - alpha

    # JOINT 2 (lower joint) ==================================

    beta = np.arccos(
        (MIDDLE_LINK_LENGTH**2 + LOWER_LINK_LENGTH**2 - tip_to_joint1_distance**2)
        / (2 * MIDDLE_LINK_LENGTH * LOWER_LINK_LENGTH)
    )
    joint2_angle = beta - np.pi

    return np.array([joint0_angle, joint1_angle, joint2_angle])


def trifingerpro_ik(tip_positions: np.ndarray) -> np.ndarray:
    """Compute inverse kinematics for the TriFingerPro.

    Args:
        tip_positions: The desired tip positions in world coordinates.

    Returns:
        Joint angles for the fingers to reach the given tip position.
    """
    angle_120 = 2 * np.pi / 3
    angle_240 = 2 * angle_120
    rotmat_z_120 = np.array(
        [
            [np.cos(angle_120), -np.sin(angle_120), 0],
            [np.sin(angle_120), np.cos(angle_120), 0],
            [0, 0, 1],
        ]
    )
    rotmat_z_240 = np.array(
        [
            [np.cos(angle_240), -np.sin(angle_240), 0],
            [np.sin(angle_240), np.cos(angle_240), 0],
            [0, 0, 1],
        ]
    )

    # rotate the tip positions of the second and third finger to align with the first
    # finger's coordinate system
    adjusted_tip_positions = [
        tip_positions[0],
        rotmat_z_120 @ np.array(tip_positions[1]).transpose(),
        rotmat_z_240 @ tip_positions[2],
    ]

    joint_positions = np.zeros(9)
    for i in range(3):
        joint_positions[i * 3 : (i + 1) * 3] = fingerpro_ik(adjusted_tip_positions[i])
    return joint_positions
