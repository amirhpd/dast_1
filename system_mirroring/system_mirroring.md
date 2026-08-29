# System Mirroring

How to bring up a machine that can build and run DAST-1. Written to be followed
top to bottom by a person or by an AI coding agent — every step has a command and
a way to check it worked.

**Target:** Ubuntu 26.04 LTS (`resolute`) with ROS 2 Lyrical Luth and Gazebo Jetty.
The project was originally Ubuntu 22.04 / ROS 2 Humble; that combination is no
longer supported here.

---

## Quick start

```bash
git clone https://github.com/amirhpd/dast_1.git
cd dast_1/system_mirroring

./install_ros2_lyrical.sh                                  # 1. ROS 2 apt source + base system
./install_ros2_packages.sh                                 # 2. ROS/system packages + rosdep
pip install --user -r installed_python_packages.txt        # 3. non-apt Python packages
./install_vscode_extensions.sh                             # 4. VS Code (optional)

cd ..
source /opt/ros/lyrical/setup.bash
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON   # 5. build
source install/setup.bash

./system_mirroring/verify_setup.sh                             # 6. check
```

`verify_setup.sh` prints `PASS`/`FAIL` per item and exits non-zero if anything
failed. A correct machine prints `ALL CHECKS PASSED`.

---

## What each file is

| File | Purpose |
|---|---|
| `install_ros2_lyrical.sh` | Locale, `universe`, the `ros2-apt-source` .deb, `apt update/upgrade` |
| `install_ros2_packages.sh` | Installs the list below, then `rosdep init && rosdep update` |
| `installed_ros2_packages.txt` | **14 top-level apt packages.** apt resolves the rest |
| `installed_ros2_packages_full.txt` | The ~415 packages that actually land, pinned to versions. Reference only — for diffing a broken machine against a working one, not for installing |
| `installed_python_packages.txt` | The one Python package that is *not* in apt (`pypcd4`) |
| `install_vscode_extensions.sh` | Installs the extensions and copies `vscode_config/` into `../.vscode/` |
| `installed_vscode_extensions.txt` | Extension IDs, one per line |
| `vscode_config/` | `settings.json` and `c_cpp_properties.json` templates |
| `verify_setup.sh` | 27 PASS/FAIL checks across environment, workspace, rosdep and VS Code |

---

## Step details and known traps

### 1. ROS 2 Lyrical

The apt-source URL embeds the Ubuntu codename, which the script expands from
`/etc/os-release`. If you run the upstream one-liner by hand, keep the space in
`. /etc/os-release` — without it bash reads `./etc/os-release`, `UBUNTU_CODENAME`
stays empty, the URL 404s, and `curl` silently writes a **9-byte** file that
`dpkg -i` then rejects. The script hard-fails on any download under 1000 bytes
for exactly this reason.

Check: `ls /opt/ros/lyrical/setup.bash` exists.

### 2. ROS and system packages

`installed_ros2_packages.txt` holds only what was installed *deliberately*
(`apt-mark showmanual`), so apt is free to resolve dependencies its own way.
Beyond ROS itself the notable entries are `libserial-dev` (the hardware
interface links against it) and `ros-lyrical-gz-ros2-control` (the Gazebo
bridge — note the `gz_`/`GazeboSim` naming, **not** the old `ign_`/`Ignition`).

`rosdep init` is idempotent-guarded in the script; it fails loudly if the
sources list already exists, so the script checks first.

Check: `rosdep install --from-paths src --ignore-src -r --simulate` from the
repo root resolves without error.

### 3. Python

Only `pypcd4` needs pip — `publish_pointcloud.py` uses it to read `points.pcd`.
`numpy`, `pyserial` and every `rclpy`/`launch`/`moveit_configs_utils` module come
from the apt packages. **Do not pip-install ROS Python modules**; they must match
the apt build. Ubuntu 26.04 is PEP 668 externally-managed, so pip needs `--user`.

### 4. VS Code

`.vscode/` is gitignored, so the working config is kept here in `vscode_config/`
and copied into place by the script.

Red squiggles under `#include` lines mean IntelliSense cannot find the ROS
headers. The fix is the compile database, not another extension:

1. Build with `--cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON`. This produces
   `build/<pkg>/compile_commands.json` for the four C++ packages — `controller`,
   `moveit`, `msgs`, `task`. The other three are Python/launch-only and get none.
2. `c_cpp_properties.json` lists those four paths explicitly.
3. Launch VS Code from a **sourced** shell so it inherits `AMENT_PREFIX_PATH`:
   `source /opt/ros/lyrical/setup.bash && source install/setup.bash && code .`
4. `Ctrl+Shift+P` → *C/C++: Reset IntelliSense Database*.

`settings.json` sets `cmake.configureOnOpen: false`. Leave it off: colcon owns
`build/`, and the CMake Tools extension will otherwise configure a single package
straight into `build/`, leaving a stray `CMakeCache.txt`, `CMakeFiles/` and
`Makefile` next to colcon's per-package directories.

The compile database is only as fresh as the last build — after adding an
`#include` for a new dependency, rebuild before IntelliSense can see it.

### 5. Build

```bash
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install/setup.bash        # in every new shell
```

Expect `7 packages finished`. All seven emit stderr warnings (CMake deprecation
notices); those are not failures.

### 6. Verify

```bash
./system_mirroring/verify_setup.sh
```

Then a live smoke test, headless so nothing opens on screen:

```bash
ros2 launch startup sim_robot.launch.py gui:=False rviz:=False
# second terminal, after ~30 s:
ros2 control list_controllers      # joint_state_broadcaster + manipulator_controller, both 'active'
ros2 action send_goal /task_server_angle msgs/action/TaskAction "task_number: 0"
```

A successful goal returns `success: true` and status `SUCCEEDED`.

To tear the sim down, kill the process group — a bare `pkill -f "gz sim"` also
matches the shell running it. Use a bracket pattern: `pkill -f "[g]z sim"`.

Hardware is not covered by any of this. The real robot additionally needs the
Nano 33 on `/dev/ttyACM0` and the user in the `dialout` group
(`sudo usermod -aG dialout $USER`, then log out and back in).

---

## Refreshing these files from a working machine

Run from a machine that is known good:

```bash
cd system_mirroring
apt-mark showmanual | grep -E '^(ros-|libserial|python3-colcon)' | sort > installed_ros2_packages.txt
dpkg -l | awk '/^ii/ && $2 ~ /^ros-lyrical-/ {print $2"="$3}' | sort > installed_ros2_packages_full.txt
code --list-extensions | sort | grep -v '^anthropic\.' > installed_vscode_extensions.txt
cp ../.vscode/settings.json ../.vscode/c_cpp_properties.json vscode_config/
```

Edit `installed_python_packages.txt` by hand — it is a deliberate short list, not
a `pip freeze`. A full freeze would pull ROS Python modules from PyPI and break
the install.
