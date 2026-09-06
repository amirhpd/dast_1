# DAST-1

A serial manipulator: 3D-printed mechanics, hobby servos, an Arduino Nano 33 BLE Sense Rev2,
and a ROS 2 stack with MoveIt 2 motion planning and a Gazebo simulation.
The hardware has 5 joints; the simulated robot has a sixth, `joint_6`, a wrist rotation with no
servo behind it.

## Hardware

* Mechanical parts are 3D printed — printable STLs and drawings are in [`Mechanics/`](Mechanics),
  and the coordinate frames are shown in [`frames.pdf`](frames.pdf)
* 5 × MG996R servos, one per joint
* Controller: Arduino Nano 33 BLE Sense Rev2, connected over USB
* Firmware is in a separate project: [dast_1_emb](https://github.com/amirhpd/dast_1_emb)
* Xbox 360 Kinect (model 1414) for depth sensing — see [Kinect](#kinect) below

## Requirements

* Ubuntu 26.04 LTS with ROS 2 Lyrical Luth (Gazebo Jetty)

## Set up on a new machine

Everything needed to stand up a fresh machine lives in
[`system_mirroring/`](system_mirroring). Pick whichever route suits you.

### By hand

Four scripts, in order, then build:

```bash
git clone https://github.com/amirhpd/dast_1.git
cd dast_1/system_mirroring

./install_ros2_lyrical.sh                              # ROS 2 apt source + base system
./install_ros2_packages.sh                             # ROS/system packages + rosdep
pip install --user -r installed_python_packages.txt    # non-apt Python packages
./install_vscode_extensions.sh                         # VS Code (optional)

cd ..
source /opt/ros/lyrical/setup.bash
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install/setup.bash

./system_mirroring/verify_setup.sh                     # prints PASS/FAIL per item
```

A correct machine ends with `ALL CHECKS PASSED`.
[`system_mirroring.md`](system_mirroring/system_mirroring.md) explains each step
and the traps worth knowing about.

### With an AI agent

If you use Claude Code or a similar agent, clone the repo, open it, and paste
this prompt:

> Set this machine up to build and run the DAST-1 project.
>
> Follow `system_mirroring/system_mirroring.md` from the top. It targets Ubuntu
> 26.04 with ROS 2 Lyrical Luth — confirm the machine matches before you start,
> and stop and tell me if it does not.
>
> Run the four install scripts in order, then build the workspace with
> `colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON`, then run
> `system_mirroring/verify_setup.sh` and keep working until it reports
> `ALL CHECKS PASSED`.
>
> Notes:
> - `source /opt/ros/lyrical/setup.bash` and `source install/setup.bash` do not
>   survive between shell invocations — re-source them in every command.
> - The install scripts need `sudo` and will prompt. Tell me when you need me to
>   type a password rather than trying to work around it.
> - Finish with a headless smoke test:
>   `ros2 launch startup sim_robot.launch.py gui:=False rviz:=False`, then check
>   `ros2 control list_controllers` shows both controllers `active` and that
>   `ros2 action send_goal /task_server_angle msgs/action/TaskAction "task_number: 0"`
>   returns `success: true`. Do not open Gazebo or RViz windows.
> - Tear the sim down with a bracket pattern (`pkill -f "[g]z sim"`) — a plain
>   `pkill -f "gz sim"` also matches the shell running it.
> - Report anything you had to change from the documented steps, so I can fold it
>   back into `system_mirroring/`.

Hardware is not covered by any of the above. For the real robot, also add
yourself to the `dialout` group (`sudo usermod -aG dialout $USER`, then log out
and back in) and plug the Nano 33 into `/dev/ttyACM0`.

## Build

Once the machine is set up, a normal rebuild is:

```bash
cd dast_1
colcon build
source install/setup.bash
```

Run `source install/setup.bash` once in every new terminal. Add
`--cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON` whenever you want editor
IntelliSense to pick up newly added includes.

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

All joints are limited to ±90°, and not every pose in that range can be reached.
The scan sweeps `joint_1`..`joint_5` with `joint_6` held at 0, so it understates the
reachable workspace of the 6-DOF description.
`points.pcd` holds the measured reachable workspace:

```bash
ros2 run moveit publish_pointcloud.py     # run from the repository root
```

It publishes to `/point_cloud`; add a **PointCloud2** display in RViz to see it.

## Kinect

An Xbox 360 Kinect (model 1414) provides colour and depth. It is driven by the `kinect`
package, which is **standalone**: it is not started by `sim_robot.launch.py` or
`run_robot.launch.py`, and nothing in the robot stack depends on it yet.

It needs its 12 V power brick as well as USB. On USB alone only the motor enumerates and the
LED blinks green — `lsusb | grep 045e` must list all three of `02b0` (motor), `02ad` (audio)
and `02ae` (camera).

```bash
ros2 launch kinect kinect.launch.py              # driver + static TF
ros2 launch kinect kinect.launch.py rviz:=True   # ...and RViz, preconfigured
```

| topic | type | contents |
| --- | --- | --- |
| `/kinect/rgb/image_raw` | `sensor_msgs/Image` | `rgb8`, 640×480, 30 Hz |
| `/kinect/depth/image_raw` | `sensor_msgs/Image` | `16UC1`, millimetres, 0 where nothing was seen |
| `/kinect/points` | `sensor_msgs/PointCloud2` | organised XYZRGB, one point per pixel |
| `/kinect/rgb/camera_info`, `/kinect/depth/camera_info` | `sensor_msgs/CameraInfo` | the calibration below |

Depth is captured as `FREENECT_DEPTH_REGISTERED`, so it is already aligned to the colour image
and one calibration covers both. The point cloud is only built while something is subscribed to
it — it is about 4.9 MB per frame.

### Calibration

The measured intrinsics for this unit live in
[`src/kinect/config/kinect_rgb.yaml`](src/kinect/config/kinect_rgb.yaml) and are loaded at
startup through the `camera_info_url` parameter
(default `package://kinect/config/kinect_rgb.yaml`). Editing that file and restarting is
enough — no rebuild. If the file is missing or is not 640×480, the node warns and falls back to
nominal Kinect v1 values, and the cloud becomes approximate.

| | |
| --- | --- |
| fx, fy | 549.996, 552.698 |
| cx, cy | 316.370, 269.456 |
| distortion (`plumb_bob`) | 0.168766, −0.361444, 0.002809, −0.002889, 0 |

Distortion is corrected when the cloud is built, through a per-pixel lookup table computed once
at startup. It is not a small effect: at 1 m depth it moves a corner pixel by about 4 cm.

To redo the calibration — 8×6 interior corners, 25 mm squares:

```bash
ros2 launch kinect kinect.launch.py
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025 \
  --no-service-check --ros-args \
  -r image:=/kinect/rgb/image_raw -r camera:=/kinect/rgb
```

Two upstream bugs in `camera_calibration` 7.1.7 have to be patched first, or it crashes on
startup and again on SAVE. Both are one-line fixes to installed files, and an `apt upgrade`
silently reverts them:

```bash
sudo sed -i 's/self\.get_logger()\.warn(/self.get_logger().warning(/' \
  /opt/ros/lyrical/lib/python3.14/site-packages/camera_calibration/camera_calibrator.py
sudo sed -i 's/\.tostring()/.tobytes()/' \
  /opt/ros/lyrical/lib/python3.14/site-packages/camera_calibration/mono_calibrator.py \
  /opt/ros/lyrical/lib/python3.14/site-packages/camera_calibration/stereo_calibrator.py
```

### Where to bolt it

Two measured limits decide the mounting position.

**Minimum range is about 50 cm.** The closest reading over 900 frames was 502 mm, and nothing
nearer than that exists. Anything closer is reported as 0, which becomes a *hole* in the point
cloud — and a hole reads as empty space to MoveIt, not as an obstacle. Mount the camera so the
whole arm workspace stays beyond roughly 60 cm.

**The field of view is 60.4° × 46.9°**, computed from the intrinsics above:

| distance | area covered |
| --- | --- |
| 0.6 m | 0.70 × 0.52 m |
| 1.0 m | 1.16 × 0.87 m |
| 1.5 m | 1.75 × 1.30 m |
| 2.0 m | 2.33 × 1.74 m |

The arm reaches about 0.5 m, so an eye-to-hand mount 1.2–1.5 m back covers the workspace with
margin and stays clear of the 50 cm floor.

Depth noise, measured as the per-pixel standard deviation over 30 s:

| range | median σ | 95th percentile |
| --- | --- | --- |
| 0.5–0.7 m | 0.5 mm | 0.7 mm |
| 0.9–1.1 m | 1.4 mm | 2.2 mm |
| 1.4–1.8 m | 2.8 mm | 4.6 mm |

A 1 cm octomap voxel is therefore comfortable out to about 2 m. Over the same 30 s the driver
delivered 899 frames at 29.97 Hz with no drops, and the Kinect had its internal USB hub to
itself; the Nano's serial traffic is negligible beside it.

**Tilt.** The Kinect's motorised base holds whatever angle it was last commanded to, and forgets
it when the 12 V drops. The driver therefore commands the tilt on every start, so it is the same
angle every run rather than whatever someone last left it at:

```bash
ros2 launch kinect kinect.launch.py tilt_degrees:=-16   # -30..30, negative is head down
```

The default is **−16°**, matching the current mount. After moving the motor the node reads the
angle back from the accelerometer and warns if it does not match what was commanded — a stalled
motor is otherwise invisible. Set `set_tilt:=False` to leave the motor untouched.

> The tilt must match the angle the camera-to-`base_link` transform was measured at. Change
> `tilt_degrees` and every extrinsic is silently wrong, with nothing to show for it but a point
> cloud that no longer lines up with the robot. Repeatability is about 1°, so bake the nominal
> tilt into the URDF as well.

### Checking the sensor

Depth and geometry come from different places, so they fail differently:

* `z` comes straight from the Kinect's own factory depth calibration and is passed through
  untouched. Check it against a tape measure on a flat wall. Expect a 1–2 cm offset, because the
  depth origin sits inside the housing rather than at the front glass — measure at two distances
  and compare the *difference*, which cancels the offset.
* `x` and `y` come from the intrinsics above. Check them against a known width: hold a ruler
  flat, facing the camera, add a **PointCloud2** display in RViz, pick the **Select** tool, box
  a few points at each end, and read `Position → x` under each point in the **Selection** panel.
  Values are in metres. Expand the `Point ...` rows — the number in the row itself is only an
  index.

## License

This project is licensed under the GPLv3 License.
