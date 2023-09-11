import pytest

from robot_properties_fingers import get_urdf_base_path

def test_get_urdf_base_path() -> None:
    # verify paths by checking for existence of one of the expected URDF files

    one = get_urdf_base_path("one")
    assert (one / "finger.urdf").is_file()

    pro = get_urdf_base_path("pro")
    assert (pro / "trifingerpro.urdf").is_file()

    edu = get_urdf_base_path("edu")
    assert (edu / "trifingeredu.urdf").is_file()

    with pytest.raises(ValueError, match="Invalid robot type foo"):
        get_urdf_base_path("foo")
