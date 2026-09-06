#!/usr/bin/env bash
#
# Kill everything a DAST-1 launch leaves behind and prove the machine is clean.
#
#   ./reset_ros.sh            stop everything, then verify
#   ./reset_ros.sh --check    verify only, kill nothing
#
# Why this exists: `ros2 launch` children are reparented to init when the launch
# dies, so Ctrl-C or a careless pkill can leave a Gazebo server running. The
# launcher is `ruby .../gz sim` but the server is `gz-sim-main`, so the obvious
# pattern "gz sim" misses it. An orphaned server still hosts the robot and its
# gz_ros2_control plugin on the same ROS domain, and the next launch then has
# two /controller_managers publishing to /joint_states at once -- the arm shows
# up in RViz at the pose the *old* Gazebo was left in, while /joint_states reads
# zeros. Duplicate node names are the tell, so that is what this checks for.

set -uo pipefail

# Full paths where possible: a loose pattern like "spawner" also matches
# unrelated system processes (gvfsd is started with --spawner), and this script
# kills what it matches.
PATTERNS=(
    "gz_sim_vendor/libexec/gz/sim[0-9]*/gz-sim-main"   # the Gazebo server itself
    "ruby .*gz_tools_vendor/bin/gz sim"                # the wrapper that starts it
    "moveit_ros_move_group/move_group"
    "install/task/lib/task/task_server_angle_node"
    "ros_gz_bridge/parameter_bridge"
    "install/kinect/lib/kinect/kinect_node"
    "robot_state_publisher/robot_state_publisher"
    "joint_state_publisher(_gui)?/joint_state_publisher"
    "controller_manager/spawner"
    "rviz2/rviz2"
    "bin/ros2 launch (startup|description|moveit|controller|task|kinect) "
)

CHECK_ONLY=false
if [[ "${1:-}" == "--check" ]]; then
    CHECK_ONLY=true
elif [[ $# -gt 0 ]]; then
    echo "usage: $0 [--check]" >&2
    exit 2
fi

# This script, the shell that started it and anything above them must never be
# killed -- a terminal's command line can contain the very patterns we match on.
ancestors()
{
    local pid=$$
    while [[ -n "$pid" && "$pid" != "0" && "$pid" != "1" ]]; do
        echo "$pid"
        pid=$(ps -o ppid= -p "$pid" 2>/dev/null | tr -d ' ')
    done
}

SAFE=$(ancestors)

# PIDs matching any pattern, minus this process tree.
targets()
{
    local pattern hits=""
    for pattern in "${PATTERNS[@]}"; do
        hits+=$(pgrep -f "$pattern" 2>/dev/null)$'\n'
    done
    echo "$hits" | grep -v '^$' | sort -u | grep -vxF "$SAFE"
}

if ! $CHECK_ONLY; then
    echo "Stopping ROS processes.."
    for signal in TERM KILL; do
        pids=$(targets)
        [[ -z "$pids" ]] && break
        kill "-$signal" $pids 2>/dev/null
        sleep 2
    done

    # The daemon caches the node graph; a stale one reports nodes that are gone.
    if command -v ros2 >/dev/null 2>&1; then
        echo "Restarting the ros2 daemon.."
        ros2 daemon stop >/dev/null 2>&1
        sleep 1
        ros2 daemon start >/dev/null 2>&1
        sleep 1
    fi
fi

status=0

echo
echo "Leftover processes:"
left=$(targets)
if [[ -z "$left" ]]; then
    echo "  none"
else
    ps -o pid=,cmd= -p $(echo "$left" | tr '\n' ' ') 2>/dev/null | cut -c1-110 | sed 's/^/  /'
    status=1
fi

echo
echo "Duplicate ROS nodes:"
if ! command -v ros2 >/dev/null 2>&1; then
    echo "  skipped -- 'ros2' not on PATH, source install/setup.bash first"
else
    duplicates=$(timeout 20 ros2 node list 2>/dev/null | sort | uniq -d)
    if [[ -z "$duplicates" ]]; then
        echo "  none"
    else
        echo "$duplicates" | sed 's/^/  /'
        status=1
    fi
fi

echo
if [[ $status -eq 0 ]]; then
    echo "Clean. Safe to launch."
else
    echo "NOT clean -- see above. Re-run ./reset_ros.sh, or kill the listed PIDs by hand."
fi
exit $status
