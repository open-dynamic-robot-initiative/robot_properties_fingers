import json
import pathlib

import numpy as np
import pytest

from robot_properties_fingers.trifingerpro_ik import trifingerpro_ik


@pytest.fixture
def test_data():
    path = pathlib.Path(__file__).parent / "trifingerpro_kinematics_data.json"
    with path.open("r") as f:
        data = json.load(f)
    return data


def test_trifingerpro_ik(test_data):
    for i in range(len(test_data["tip_positions"])):
        tip_positions = test_data["tip_positions"][i]
        expected_joint_positions = test_data["joint_positions"][i]

        computed_joint_positions = trifingerpro_ik(tip_positions)
        np.testing.assert_array_almost_equal(
            computed_joint_positions, expected_joint_positions
        )
