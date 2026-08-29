#!/bin/bash
# Step 6 -- check the machine is actually ready. Prints PASS/FAIL per item and
# exits non-zero if anything failed. Run from anywhere; sources the workspace
# itself. Does NOT need hardware.
WS="$(cd "$(dirname "$0")/.." && pwd)"
FAILED=0

check() {  # check <label> <command...>
    local label="$1"; shift
    if "$@" >/dev/null 2>&1; then
        printf 'PASS  %s\n' "${label}"
    else
        printf 'FAIL  %s\n' "${label}"
        FAILED=1
    fi
}

echo "--- environment ---"
check "Ubuntu 26.04"            bash -c '. /etc/os-release; [ "$VERSION_ID" = 26.04 ]'
check "ROS 2 Lyrical present"   test -f /opt/ros/lyrical/setup.bash
check "colcon on PATH"          command -v colcon
check "rosdep initialised"      test -f /etc/ros/rosdep/sources.list.d/20-default.list
check "Gazebo (gz) on PATH"     command -v gz
check "libserial headers"       test -f /usr/include/libserial/SerialPort.h
check "pypcd4 importable"       python3 -c 'import pypcd4'

echo "--- workspace ---"
check "workspace built"         test -f "${WS}/install/setup.bash"
for p in controller description micro moveit msgs startup task; do
    check "package '${p}' installed" test -d "${WS}/install/${p}"
done
check "compile database present" bash -c "ls ${WS}/build/*/compile_commands.json"

echo "--- rosdep resolves every package.xml ---"
if (. /opt/ros/lyrical/setup.bash && cd "${WS}" && rosdep install --from-paths src --ignore-src -r --simulate) >/dev/null 2>&1; then
    printf 'PASS  all declared dependencies satisfiable\n'
else
    printf 'FAIL  all declared dependencies satisfiable\n'
    FAILED=1
fi

echo "--- vs code ---"
check "settings.json in place"        test -f "${WS}/.vscode/settings.json"
check "c_cpp_properties.json in place" test -f "${WS}/.vscode/c_cpp_properties.json"
if command -v code >/dev/null 2>&1; then
    INSTALLED="$(code --list-extensions 2>/dev/null)"
    while read -r ext; do
        if grep -qix "${ext}" <<<"${INSTALLED}"; then
            printf 'PASS  extension %s\n' "${ext}"
        else
            printf 'FAIL  extension %s\n' "${ext}"
            FAILED=1
        fi
    done < <(grep -vE '^\s*(#|$)' "$(dirname "$0")/installed_vscode_extensions.txt")
else
    printf 'SKIP  VS Code CLI not on PATH\n'
fi

echo
if [ "${FAILED}" -eq 0 ]; then
    echo "ALL CHECKS PASSED"
else
    echo "SOME CHECKS FAILED"
fi
exit "${FAILED}"
