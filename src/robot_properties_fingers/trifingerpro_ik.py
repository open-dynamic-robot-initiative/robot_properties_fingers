#!/usr/bin/env python3
"""TriFingerPro Inverse Kinematics."""

import numpy as np

# from the URDF model
BASE_HEIGHT = 0.29
TIP_X_OFFSET = 0.086
MIDDLE_LINK_LENGTH = 0.16
LOWER_LINK_LENGTH = 0.16
JOINT1_Y_OFFSET = 0.0505


def get_tangent_points(px, py, cx, cy, radius):
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
    """Compute inverse kinematics for FingerPro."""
    # TODO: check if this actually works for the single finger model or if the origin is
    # different there!
    tip_position = np.asarray(tip_position)

    # JOINT 0 ==================================
    tip_xz = tip_position[[0, 2]]

    tangent_points = get_tangent_points(
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

    # JOINT 1 ==================================

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

    # alternative: rotate tip-to_joint_on_plane to align with z-azis
    tip_to_joint1_foo = rot_y_inv @ tip_to_joint1
    # normalize for angle calculation
    tip_to_joint1_foo /= np.linalg.norm(tip_to_joint1_foo)
    alpha = np.pi / 2 - np.arctan2(tip_to_joint1_foo[2], tip_to_joint1_foo[1])

    alpha_and_q1 = np.arccos(
        (tip_to_joint1_distance**2 + MIDDLE_LINK_LENGTH**2 - LOWER_LINK_LENGTH**2)
        / (2 * tip_to_joint1_distance * MIDDLE_LINK_LENGTH)
    )
    joint1_angle = alpha_and_q1 - alpha

    # JOINT 1 ==================================

    beta = np.arccos(
        (MIDDLE_LINK_LENGTH**2 + LOWER_LINK_LENGTH**2 - tip_to_joint1_distance**2)
        / (2 * MIDDLE_LINK_LENGTH * LOWER_LINK_LENGTH)
    )
    joint2_angle = beta - np.pi

    return np.array([joint0_angle, joint1_angle, joint2_angle])


def trifingerpro_ik(tip_positions: np.ndarray) -> np.ndarray:
    """Compute inverse kinematics for all three fingers of the FingerPro."""
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
