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

Give it half a minute to settle. `ros2 control list_controllers` should show both
`joint_state_broadcaster` and `manipulator_controller` as `active`.

Everything below runs from a **second terminal**, sourced the same way
(`cd dast_1 && source install/setup.bash`).

### Moving the arm

There are two families of commands. The first plans through MoveIt: motions are checked
against obstacles and the arm takes a safe route. The second talks straight to the
controller: faster and simpler, but nothing is checked before the arm moves.

#### Through MoveIt — collision-checked

**Drag it in RViz.** In the **MotionPlanning** panel, drag the goal marker to where you
want the tip, then press *Plan & Execute*.

**Run a stored task.** Three canned poses ship with the project, numbered 0, 1 and 2:

```bash
ros2 action send_goal /task_server_angle msgs/action/TaskAction "task_number: 1"
```

**Set the joint angles** — five values, in **degrees**, one per joint:

```bash
ros2 run moveit interface_angle 20 30 -15 40 10
```

**Set the tip pose** — `x y z roll pitch yaw`, in **metres and radians**:

```bash
ros2 run moveit interface_pose 1.173 -3.223 6.196 0.953 -0.143 0.450
```

Not every pose is reachable; see *Reachable workspace* below. To find a valid one, move the
arm with `interface_angle` and read the tip off `ros2 run tf2_ros tf2_echo world tip`.

**Follow a trajectory of several waypoints.** The waypoints live in a YAML file, and the
whole path is planned segment by segment:

```bash
ros2 run moveit interface_sequence src/moveit/config/sequence_example.yaml
```

Copy `src/moveit/config/sequence_example.yaml` and edit its `segments:` list to make your
own path. Each segment takes either `joints:` (radians) or `pose:`, and the file's comments
explain the rest. Leave `blend_radius: 0` — corner-rounding needs a 6-DOF arm and DAST-1
has 5, so any other value fails the whole sequence.

#### Straight to the controller — no collision checking

These bypass MoveIt entirely. Nothing stops the arm driving through an obstacle, or through
itself, so keep the workspace clear.

**Send one trajectory and forget it.** Positions are in **radians**, in `joint_names` order:

```bash
ros2 topic pub --once /manipulator_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
'{joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5],
  points: [{positions: [0.3, 0.4, -0.2, 0.5, 0.1], time_from_start: {sec: 3}}]}'
```

**Send it and wait for the result.** Same thing as an action, so you get success or failure
back, and `--feedback` will stream progress while it runs:

```bash
ros2 action send_goal /manipulator_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
'{trajectory: {joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5],
  points: [{positions: [0.4, 0.2, 0.3, 0.5, 0.0], time_from_start: {sec: 2}},
           {positions: [-0.4, 0.6, 0.1, 1.0, 0.2], time_from_start: {sec: 5}}]}}'
```

`time_from_start` counts from the beginning of the trajectory, so each waypoint's value must
be larger than the one before it.

### Reachable workspace

All five joints are limited to ±90°, and not every pose in that range can be reached.
`points.pcd` holds the measured reachable workspace:

```bash
ros2 run moveit publish_pointcloud.py     # run from the repository root
```

It publishes to `/point_cloud`; add a **PointCloud2** display in RViz to see it.

## License

This project is licensed under the GPLv3 License.
