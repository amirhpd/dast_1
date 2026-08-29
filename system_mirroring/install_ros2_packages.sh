#!/bin/bash
# Step 2 -- install the ROS 2 and system packages the project needs.
# Reads installed_ros2_packages.txt: the 14 top-level packages, from which apt
# resolves the ~400 dependencies. Safe to re-run.
set -euo pipefail

cd "$(dirname "$0")"
LIST="installed_ros2_packages.txt"
[ -f "${LIST}" ] || { echo "ERROR: ${LIST} not found." >&2; exit 1; }

mapfile -t PACKAGES < <(grep -vE '^\s*(#|$)' "${LIST}")
echo "Installing ${#PACKAGES[@]} top-level packages..."

sudo apt update
sudo apt install -y "${PACKAGES[@]}"

# rosdep resolves the <depend> tags in each package.xml to apt names.
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

echo
echo "Done. Next: pip install --user -r installed_python_packages.txt"
