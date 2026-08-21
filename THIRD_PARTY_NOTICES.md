# Third-party notices

## Universal Robots Gazebo simulation

The combined Gazebo Harmonic Xacro descriptions and the local simulation launch
in `ur_cbf_bringup` are based on the structure of
[`Universal_Robots_ROS2_GZ_Simulation`](https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation),
commit `048c80cd1faf87a2c74e14baadb65bd22b564d8f`. That project is distributed
under the BSD 3-Clause License. The local launch retains the upstream copyright
and license header.

## OnRobot RG2/RG6 descriptions

The Docker image installs the meshes and Xacro macros from
[`tonydle/OnRobot_ROS2_Description`](https://github.com/tonydle/OnRobot_ROS2_Description),
commit `29180b3fa9cba6555f3e515e789b8ccd34252fab`. That project is distributed
under the MIT License. Its source files are not vendored in this repository.
During the image build, the URDF `mimic` tags are removed from the installed
copy so the local width adapter can command every simulated joint explicitly.

## OnRobot RG2/RG6 hardware driver

The Docker image also builds
[`tonydle/OnRobot_ROS2_Driver`](https://github.com/tonydle/OnRobot_ROS2_Driver).
The revision is pinned to commit
`b99abaccfbbe90f2096feff833f4c0849757a587` and provides the Modbus backend
used by the real-hardware profile. That project is distributed under the MIT
License. Its source is downloaded during the image build and is not vendored in
this repository.
