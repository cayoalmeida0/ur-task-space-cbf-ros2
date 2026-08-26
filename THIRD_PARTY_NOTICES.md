# Third-party notices

## UAIbot UR3e collision geometry

The thirteen arm and wrist primitive types, dimensions and homogeneous
transforms used by `cbf_visual_volumes.urdf.xacro` are derived from
[`UAIbot/UAIbotPy`](https://github.com/UAIbot/UAIbotPy/blob/1acb5ed637738aca4ea05945e6c065c3757bc13d/uaibot/robot/_create_ur_ur3e.py),
commit `1acb5ed637738aca4ea05945e6c065c3757bc13d`. The transforms were converted
from the UAIbot DH frames to the corresponding ROS 2 Jazzy URDF link frames.
The project copy preserves the primitive types, rotations, and dimensions of
all thirteen upstream UR3e arm objects. Revision 0.6.9 sets the URDF-frame x/z
of `c21` to 0/0.020 m; z of `c22` to 0.0225 m; preserves z=0.025 m in `c23`;
sets x/y/z of `c31` to 0/0/0.020 m and x/y of `c32` to zero; preserves the
`c51` origin at (0, 0, -0.020) m; and sets z of `c52` to -0.015 m. The generic upstream
gripper geometry is replaced. The project-owned distance evaluator uses
UAIbot's public `Utils.compute_dist` implementation but replaces the
incompatible three-value unpacking in UAIbot 1.2.7
`_compute_dist_auto_python`.
The three-object RG2 capsule is original project configuration and replaces the
generic gripper geometry from the upstream factory at runtime.
UAIbot is distributed under the MIT License:

> Copyright (c) 2023 UAIbot
>
> Permission is hereby granted, free of charge, to any person obtaining a copy
> of this software and associated documentation files (the "Software"), to deal
> in the Software without restriction, including without limitation the rights
> to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
> copies of the Software, and to permit persons to whom the Software is
> furnished to do so, subject to the following conditions:
>
> The above copyright notice and this permission notice shall be included in all
> copies or substantial portions of the Software.
>
> THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
> IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
> FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
> AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
> LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
> OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
> SOFTWARE.

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
