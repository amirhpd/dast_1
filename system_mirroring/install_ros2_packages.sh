#!/bin/bash

# Update package list
sudo apt update

# Install ROS 2 packages from the saved list
while read package; do
    sudo apt install -y "$(echo "$package" | awk '{print $1}')"
done < installed_ros2_packages.txt