# DAST-1

A 5-DOF serial manipulator: 3D-printed mechanics, hobby servos, an Arduino Nano 33 BLE Sense Rev2,
and a ROS 2 stack with MoveIt 2 motion planning and a Gazebo simulation.

## Hardware

* Mechanical parts are 3D printed — printable STLs and drawings are in [`Mechanics/`](Mechanics),
  and the coordinate frames are shown in [`frames.pdf`](frames.pdf)
* 5 × MG996R servos, one per joint
* Controller: Arduino Nano 33 BLE Sense Rev2, connected over USB
* Firmware is in a separate project: [dast_1_emb](https://github.com/amirhpd/dast_1_emb)

## Requirements

* Ubuntu 26.04 LTS with ROS 2 Lyrical Luth (Gazebo Jetty)
* See [`system_mirroring/system_mirroring.md`](system_mirroring/system_mirroring.md) to install every
  ROS and Python package the project needs

## Build

```bash
git clone https://github.com/amirhpd/dast_1.git
cd dast_1
colcon build
source install/setup.bash
```

Run `source install/setup.bash` once in every new terminal.

## Run

Simulation — Gazebo, MoveIt and RViz, no hardware needed:

```bash
ros2 launch startup sim_robot.launch.py
```

Real robot — plug the Nano 33 in first (it is expected on `/dev/ttyACM0`):

```bash
ros2 launch startup run_robot.launch.py
```

Either way, drag the arm to a goal in the RViz **MotionPlanning** panel and press *Plan & Execute*,
or send commands from a second terminal:

```bash
# move each joint to an angle, in degrees
ros2 run moveit interface_angle 60 30 90 90 0

# move the gripper tip to a pose: x y z roll pitch yaw
ros2 run moveit interface_pose 2.094 -1.345 0.038 -3.053 0.0 1.0

# run a stored task (0, 1 or 2)
ros2 action send_goal /task_server_angle msgs/action/TaskAction "task_number: 0"
```

All five joints are limited to ±90°. Not every pose is reachable — `points.pcd` holds the
measured reachable workspace, and `ros2 run moveit publish_pointcloud.py` displays it in RViz
(run it from the repository root so it finds the file).

## License

This project is licensed under the GPLv3 License.
