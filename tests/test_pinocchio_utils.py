import pytest
import numpy as np

import robot_properties_fingers as pf


@pytest.fixture()
def kinematics() -> pf.Kinematics:
    urdf_file = pf.get_urdf_base_path("pro") / "trifingerpro.urdf"

    return pf.Kinematics(
        urdf_file,
        ["finger_tip_link_0", "finger_tip_link_120", "finger_tip_link_240"],
    )


def test_forward_kinematics_position_only(kinematics) -> None:
    tip_positions = kinematics.forward_kinematics(np.array([0, 0.9, -1.7] * 3))
    np.testing.assert_array_almost_equal(
        tip_positions,
        [
            [0.086, 0.061055, 0.079069],
            [0.009875, -0.105006, 0.079069],
            [-0.095875, 0.043951, 0.079069],
        ],
    )


def test_forward_kinematics_velocity_zero(kinematics) -> None:
    tip_positions, tip_velocities = kinematics.forward_kinematics(
        np.array([0, 0.9, -1.7] * 3), np.zeros(9)
    )
    np.testing.assert_array_almost_equal(
        tip_positions,
        [
            [0.086, 0.061055, 0.079069],
            [0.009875, -0.105006, 0.079069],
            [-0.095875, 0.043951, 0.079069],
        ],
    )
    np.testing.assert_array_almost_equal(
        tip_velocities,
        [[0, 0, 0], [0, 0, 0], [0, 0, 0]],
    )


def test_forward_kinematics_velocity_nonzero(kinematics) -> None:
    tip_positions, tip_velocities = kinematics.forward_kinematics(
        np.zeros(9), np.array([0, 0, 0.5, 0, 0, 0, 1.0, 0.3, -0.1])
    )
    np.testing.assert_array_almost_equal(
        tip_positions,
        [
            [0.086, 0.0505, -0.03],
            [0.000734, -0.099728, -0.03],
            [-0.086734, 0.049228, -0.03],
        ],
    )
    np.testing.assert_array_almost_equal(
        tip_velocities,
        [[0.0, 0.5 * 0.16, 0.0], [0.0, 0.0, 0.0], [0.090718, -0.317128, -0.086]],
    )


def test_inverse_kinematics(kinematics) -> None:
    tip_positions = np.array(
        [
            [0.086, 0.061055, 0.079069],
            [0.009875, -0.105006, 0.079069],
            [-0.095875, 0.043951, 0.079069],
        ]
    )

    tolerance = 0.001
    # need a initial guess that goes in the right direction to avoid solution where the
    # "elbow" points in the other direction
    initial_guess = np.array([0, 0.3, -1.0] * 3)
    joint_angles, error = kinematics.inverse_kinematics(
        tip_positions, initial_guess, tolerance=tolerance
    )
    assert np.all(np.array(error) < tolerance), "Error exceeds tolerance"
    # set decimal to within the tolerance
    np.testing.assert_array_almost_equal(
        joint_angles, np.array([0, 0.9, -1.7] * 3), decimal=2
    )
