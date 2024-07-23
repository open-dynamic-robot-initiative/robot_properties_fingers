"""Setup script for the robot_properties_fingers package."""

import pathlib
from os import path

from setuptools import find_packages, setup
from setuptools.command.build_py import build_py

import xacro

package_name = "robot_properties_fingers"


class CustomBuildCommand(build_py):
    """Custom build command to process xacro files."""

    def run(self) -> None:
        """Run the build command."""
        # Call the original build_py command
        build_py.run(self)

        # Define the paths
        xacro_dir = pathlib.Path("xacro_files")
        urdf_dir = pathlib.Path(self.build_lib) / package_name / "urdf"

        # Process each xacro file
        for xacro_file in xacro_dir.glob("**/*.xacro"):
            relative_urdf_path = xacro_file.relative_to(xacro_dir).with_suffix("")
            urdf_file = urdf_dir / relative_urdf_path

            # ensure output directory exists
            urdf_file.parent.mkdir(parents=True, exist_ok=True)

            # Use xacro API to process the file
            doc = xacro.process_file(str(xacro_file))
            urdf_content = doc.toprettyxml(indent="  ")
            urdf_file.write_text(urdf_content)


setup(
    name=package_name,
    version="1.2.0",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    package_data={
        "robot_properties_fingers": ["meshes/*.stl", "meshes/**/*.stl"]
    },
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [path.join("resource", package_name)],
        ),
        (
            path.join("share", package_name),
            ["package.xml"],
        ),
    ],
    install_requires=[
        "setuptools",
        "xacro",
        "pin",  # pinocchio
    ],
    zip_safe=True,
    maintainer="Felix Kloss",
    maintainer_email="felix.kloss@tue.mpg.de",
    description="Xacro files and meshes of the (Tri-)Finger robots.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
    cmdclass={"build_py": CustomBuildCommand},
)
