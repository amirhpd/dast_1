---
name: after-cad-change
description: Bring DAST-1 back in sync after the user changes the CAD — new or re-exported STLs, changed joint offsets/axes or mesh rotations in description.urdf.xacro, or edited mechlint.yaml. Regenerates inertials and the torque budget, fixes mechlint.yaml comments, recomputes pose waypoints, verifies in the headless sim, regenerates frames.pdf and updates the docs. Use when the user says they changed STLs, the CAD, the geometry, or asks to "update everything".
---

# After a CAD change

Everything below is derived from three sources of truth. Never edit them to make a result
look right — ask the user instead:

- `src/description/meshes/*.stl` (the user's CATIA exports)
- `src/description/urdf/description.urdf.xacro` (offsets, axes, mesh `rpy`)
- `mechlint.yaml` (drives, voltages, servo `at:` positions)

Run from the repo root. `M="uv run --project ../mechlint mechlint"`. Scratch files go in the
session scratchpad, not the repo. Never commit.

## 1. See what changed

```bash
git status --short; git diff --stat
git diff src/description/urdf/description.urdf.xacro mechlint.yaml
```

Note which STLs, joint offsets, joint axes and mesh `rpy` changed. Then decide:

- **A mesh that carries a servo changed** (the links listed under `components:` in
  `mechlint.yaml`) and `mechlint.yaml` did not: the servo `at:` positions are probably stale.
  Ask the user to re-measure them in CATIA. Do not guess.
- **A mesh `rpy` changed on a link that carries a servo:** the `at:` values must be turned
  with it. `at` = R · (CATIA x, y, z) / 100, where R is the `rpy` matrix (fixed axes, roll then
  pitch then yaw). Verify by checking that the link's centre of mass from `$M inertia` moved by
  the same rotation.
- **Rotation requests:** ask for an axis mapping ("CATIA x points along URDF -z, ...") rather
  than "rotate by 90 on x then z" — CATIA rotates about the part's moving axes, URDF `rpy` about
  fixed axes, so the words are ambiguous. Check the resulting matrix with numpy.

## 2. mechlint

```bash
$M inertia -w src/description/urdf/        # regenerate inertials.xacro
$M urdf-check                               # U007, M001, U006, D00x
$M torque -P 0 -P 100 -P 200                # the table
$M torque -P 0 -P 100 -P 200 -f json        # worst poses, downstream_mass_kg, motor_required_Nm
```

The mechlint MCP server may hold stale code; use the CLI. Every number in the docs comes from
these outputs, never from your own arithmetic. If a joint fails, run
`$M actuators -t <motor_required_Nm> -V <supply_V>` to list what would fit.

## 3. `mechlint.yaml` comments

Each component line ends with `# parent side; on axis [...]` or `# child side; on axis [0, 0, 0]`.
For a parent-side servo, "on axis" is the `xyz` of the joint it drives, from the xacro. Fix any
that drifted. Also check the `voltage:` comment lists the actual drives on the default rail, and
that the CATIA-conversion comment matches the current mesh rotations. Don't change values.

## 4. Build and validate

```bash
source /opt/ros/*/setup.bash && colcon build && source install/setup.bash
xacro src/description/urdf/description.urdf.xacro is_sim:=true > $SCRATCH/d.urdf
check_urdf $SCRATCH/d.urdf
```

## 5. Pose waypoints

`sequence_example_2.yaml` stores poses computed from the joint sets in its `# from joints`
comments. Recompute them:

```bash
python3 .claude/skills/after-cad-change/scripts/fk_poses.py $SCRATCH/d.urdf \
    src/moveit/config/sequence_example_2.yaml --write
```

Then set the `interface_pose` example in `README.md` (section *Set the tip pose*) to the first
pose.

## 6. Headless sim

```bash
./reset_ros.sh
setsid ros2 launch startup sim_robot.launch.py gui:=False rviz:=False > $SCRATCH/sim.log 2>&1 &
# wait until manipulator_controller is active and /task_server_angle exists
```

1. `timeout 5 ros2 run tf2_ros tf2_echo world tip -p 6` must equal the script's
   "zero pose tip". If not, the script or the URDF is wrong — stop and find out why.
2. Before **each** move: `ros2 service call /clear_octomap std_srvs/srv/Empty` (the simulated
   Kinect leaves the arm in the octomap).
3. Task goals 0, 1, 2: `ros2 action send_goal /task_server_angle msgs/action/TaskAction "task_number: N"`,
   record the tip position after each.
4. `ros2 run moveit interface_sequence src/moveit/config/sequence_example.yaml`, then `_2.yaml`.
   The `tip_group` error line is pre-existing and harmless.
5. `frames.pdf`: `cd $SCRATCH && ros2 run tf2_tools view_frames -o frames`, then copy
   `$SCRATCH/frames.pdf` over `frames.pdf`. It shows the frame tree only, not axis directions.
6. Tear down: `pkill -INT -f "[r]os2 launch startup sim_robot"`, then `./reset_ros.sh` must
   report clean.

The task vectors in `src/task/src/*.cpp` are joint targets: they keep planning after a geometry
change but land somewhere else. Report where; retarget only if the user gives target poses.

## 7. Docs

- `Mechanics/torque_budget.md`: rewrite with today's date. Keep the sections: question, answer
  (one-line verdict + table of torque and margin per payload), worst poses, mass each joint
  carries, caveats (static, `low` confidence, M001 hulls, Kinect not carried), U007, reproduce.
- `CLAUDE.md` (*The 6-DOF design* section): joint axes in the chain diagram, links with rotated
  meshes, zero-pose tip, M001 list, torque summary.
- `README.md`: Hardware drive list, reach (from U001: "reach to tip is N units"), *Units*
  section, `interface_pose` example.
- `todo_6dof.md`: tick off what is done, update the M001 and U007 items.
- `description.urdf.xacro`: only the reach number in the Kinect comment. Nothing else.
- grep for leftovers: `grep -rn -i "<old reach>\|<old tip>\|<removed actuator>" README.md CLAUDE.md todo_6dof.md`

## 8. Report

Short: what was regenerated, the torque verdict (weakest joints with margins), sim results
(pass/fail per move, task tip positions), anything that needs the user (stale `at:` values,
non-watertight meshes, retargeting). Say the changes are ready to commit; do not commit.
