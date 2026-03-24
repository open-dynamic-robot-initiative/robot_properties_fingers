******************************
Forward and Inverse Kinematics
******************************

Apart from the URDF models, this package provides functions for computing the forward
and inverse kinematics for the robot finger tips.

Generic
=======

The :class:`~robot_properties_fingers.Kinematics` class provides generic :func:`forward <robot_properties_fingers.Kinematics.forward_kinematics>` and
:func:`inverse <robot_properties_fingers.Kinematics.inverse_kinematics>` kinematics based on the robot's URDF file.

It's :func:`~robot_properties_fingers.Kinematics.inverse_kinematics` function tries to
find joint angles using an iterative optimization.  It needs an initial guess as
argument and is rather sensible to it, i.e. a bad initial guess can result in a bad
solution.

TriFingerPro
============

For the FingerPro and TriFingerPro, there are special implementations of the inverse
kinematics which directly compute a solution based on the known kinematics of the robot
(:func:`~robot_properties_fingers.trifingerpro_ik.fingerpro_ik`,
:func:`~robot_properties_fingers.trifingerpro_ik.trifingerpro_ik`).
Those are recommended over the generic implementation mentioned above, as it is
magnitudes faster and always returns an accurate solution (assuming that the desired tip
positions are actually reachable).
