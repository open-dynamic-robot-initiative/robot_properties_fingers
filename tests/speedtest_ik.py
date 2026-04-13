"""Compare execution speed of trifingerpro_ik and Kinematics.inverse_kinematics."""

import json
import pathlib
import timeit

import numpy as np

import robot_properties_fingers as pf
from robot_properties_fingers.trifingerpro_ik import trifingerpro_ik


def load_test_data():
    path = pathlib.Path(__file__).parent / "trifingerpro_kinematics_data.json"
    with path.open("r") as f:
        data = json.load(f)
    return data


def init_kinematics() -> pf.Kinematics:
    urdf_file = pf.get_urdf_base_path("pro") / "trifingerpro.urdf"

    return pf.Kinematics(
        urdf_file,
        ["finger_tip_link_0", "finger_tip_link_120", "finger_tip_link_240"],
    )


def test_trifingerpro_ik(test_data):
    for i in range(len(test_data["tip_positions"])):
        trifingerpro_ik(test_data["tip_positions"][i])


def test_inverse_kinematics(kinematics, test_data) -> None:
    tolerance = 0.001
    # need a initial guess that goes in the right direction to avoid solution where
    # the "elbow" points in the other direction
    initial_guess = np.array([0, 0.3, -1.0] * 3)

    for i in range(len(test_data["tip_positions"])):
        joint_angles, error = kinematics.inverse_kinematics(
            test_data["tip_positions"][i], initial_guess, tolerance=tolerance
        )


if __name__ == "__main__":
    kinematics = init_kinematics()
    test_data = load_test_data()
    n_samples = len(test_data["tip_positions"])

    duration = timeit.timeit(lambda: test_trifingerpro_ik(test_data), number=1)
    duration /= n_samples
    duration *= 1000  # convert to milliseconds
    print(f"trifingerpro_ik: {duration:.2f} ms")

    duration = timeit.timeit(
        lambda: test_inverse_kinematics(kinematics, test_data), number=1
    )
    duration /= n_samples
    duration *= 1000  # convert to milliseconds
    print(f"Kinematics.inverse_kinematics: {duration:.2f} ms")
