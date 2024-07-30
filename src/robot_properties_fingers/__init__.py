"""Robot models and related utilities for the (Tri-)Finger robots."""

import pathlib

from .pinocchio_utils import Kinematics


def get_urdf_base_path(robot_type: str) -> pathlib.Path:
    """Get path to the directory in which the URDF files are stored.

    Example to get the path to the TriFingerPro URDF:

    .. code-block:: Python

         urdf_path = get_urdf_base_path("pro") / "trifingerpro.urdf"

    Args:
        robot_type: Type of the robot.  One of {"pro", "edu", "one", "_root_"}.
            "_root_" is a special type that returns the root directory containing all
            the URDF files.

    Returns:
        Path to the directory containing the URDF files of the specified robot type.
    """
    ROBOT_TYPES = ("pro", "edu", "one", "_root_")
    if robot_type not in ROBOT_TYPES:
        raise ValueError(
            f"Invalid robot type {robot_type}.  Valid types are {ROBOT_TYPES}."
        )

    base_dir = pathlib.Path(__file__).parent / "urdf"

    if robot_type in ("_root_", "one"):
        return base_dir
    return base_dir / robot_type


__all__ = ("Kinematics", "get_urdf_base_path")
