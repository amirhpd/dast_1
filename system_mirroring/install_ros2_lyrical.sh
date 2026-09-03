#!/bin/bash
# Step 1 -- install ROS 2 Lyrical Luth on Ubuntu 26.04 (resolute).
# Safe to re-run; every step is idempotent.
set -euo pipefail

. /etc/os-release
if [ "${VERSION_ID}" != "26.04" ]; then
    echo "ERROR: this script targets Ubuntu 26.04, found ${VERSION_ID}." >&2
    echo "ROS 2 Lyrical Luth has no build for ${UBUNTU_CODENAME}." >&2
    exit 1
fi

sudo apt update
sudo apt install -y curl ca-certificates gnupg locales software-properties-common

# ROS needs a UTF-8 locale.
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

sudo add-apt-repository -y universe

# The ros2-apt-source .deb carries the key and the apt list file.
# NOTE: the codename must be expanded from /etc/os-release. If UBUNTU_CODENAME
# is empty the URL 404s and curl writes a 9-byte error page -- hence the size
# check below.
ROS_APT_SOURCE_VERSION="$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
    | grep -F '"tag_name"' | awk -F'"' '{print $4}')"
DEB="/tmp/ros2-apt-source.deb"
curl -fsSL -o "${DEB}" \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${UBUNTU_CODENAME}_all.deb"

if [ "$(stat -c%s "${DEB}")" -lt 1000 ]; then
    echo "ERROR: ${DEB} is only $(stat -c%s "${DEB}") bytes -- the download failed." >&2
    echo "Check that UBUNTU_CODENAME resolved (got '${UBUNTU_CODENAME}')." >&2
    exit 1
fi
sudo dpkg -i "${DEB}"

sudo apt update
sudo apt upgrade -y

echo
echo "ROS 2 apt source installed. Next: ./install_ros2_packages.sh"
