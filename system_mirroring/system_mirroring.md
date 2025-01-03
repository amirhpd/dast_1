## System Mirroring
Here is the procedure to replicate the working system on another computer by installing all required packages.

#### To install necessary packages on a new computer:

1. Install **ROS2-Humble** on **Ubuntu 22.04 LTS**

2. Run the _install_ros_packages.sh_ to install ROS2 packages:

    ```
    chmod +x install_ros_packages.sh
    ./install_ros_packages.sh
    ```

3. Install Python packages:
    ```
    pip install -r installed_python_packages.txt
    ```

#### To rebuild the prepared Workspace:

1. Extract the archive file:
    ```
    tar -xzf dast_1_ws.tar.gz
    ```

2. Rebuild the workspace on the target system:
    ```
    cd somewhere_on_your_computer/dast_1_ws
    colcon build
    ```

#### How to fetch the installed packages from the host system
This is how the installed packages can be fetched and stored from a system with the working the workspace:

1. Fetch the List of Installed ROS 2 Packages:
    ```
    dpkg --get-selections | grep ros- > installed_ros2_packages.txt
    ```
2. Fetch the List of Installed Python Packages:
    ```
    pip freeze > installed_python_packages.txt
    ```
3. Backup the Workspace:
    ```
    tar -czf dast_1_ws.tar.gz path_to_working_workspace/dast_1_ws
    ```