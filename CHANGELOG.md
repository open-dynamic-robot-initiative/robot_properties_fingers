# Changelog

## [Unreleased]

## [2.0.0] - 2024-07-30
### Changed
- Converted to pure Python package.  You can still use the package in a colcon workspace
  as before, but it is now also possible to install it with pip.
- **BREAKING:** Model files are not installed to `share/` anymore.  Instead they are
  installed as package data inside the Python package.  This allows to get the
  path to the installed files in the same way with both a colcon-based and a pip-based
  installation.

### Added
- Python package with function `get_urdf_base_path()`
- Moved Pinocchio-based `Kinematics` class from trifinger_simulation to this package.

### Fixed
- Removed duplicate inertial from FingerPro tip link.

## [1.1.0] - 2022-06-28

There is no changelog for this or earlier versions.

---

[Unreleased]: https://github.com/open-dynamic-robot-initiative/robot_properties_fingers/compare/v2.0.0...HEAD
[2.0.0]: https://github.com/open-dynamic-robot-initiative/robot_properties_fingers/compare/v1.1.0...v2.0.0
[1.1.0]: https://github.com/open-dynamic-robot-initiative/robot_properties_fingers/releases/tag/v1.1.0
