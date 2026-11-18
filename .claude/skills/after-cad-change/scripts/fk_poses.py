"""Forward kinematics for the pose waypoints of a sequence YAML.

Reads the expanded URDF and every "# from joints a, b, c, d, e, f" comment in the
sequence file, and computes the `tip` pose (x y z roll pitch yaw, URDF units and
radians) for each. With --write, replaces the `pose:` line that follows each such
comment. Without it, only prints.

    xacro src/description/urdf/description.urdf.xacro is_sim:=true > /tmp/d.urdf
    python3 fk_poses.py /tmp/d.urdf src/moveit/config/sequence_example_2.yaml [--write]

Also prints the tip at all-zero joints, to compare against
`ros2 run tf2_ros tf2_echo world tip -p 6`.
"""
import re
import sys
import xml.etree.ElementTree as ET

import numpy as np


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx  # URDF: fixed axes, roll then pitch then yaw


def axis_angle(axis, angle):
    k = np.asarray(axis, float)
    k = k / np.linalg.norm(k)
    kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(angle) * kx + (1 - np.cos(angle)) * kx @ kx


def matrix_rpy(m):
    pitch = -np.arcsin(np.clip(m[2, 0], -1, 1))
    if abs(np.cos(pitch)) < 1e-9:  # gimbal lock: put everything in yaw
        return 0.0, pitch, np.arctan2(-m[0, 1], m[1, 1])
    return np.arctan2(m[2, 1], m[2, 2]), pitch, np.arctan2(m[1, 0], m[0, 0])


def floats(s, default="0 0 0"):
    return [float(v) for v in (s or default).split()]


def fk(urdf_root, q, link="tip"):
    by_child = {j.find("child").get("link"): j for j in urdf_root.findall("joint")}
    chain = []
    while link in by_child:
        j = by_child[link]
        chain.append(j)
        link = j.find("parent").get("link")
    t = np.eye(4)
    for j in reversed(chain):
        o = j.find("origin")
        step = np.eye(4)
        step[:3, :3] = rpy_matrix(*floats(o.get("rpy") if o is not None else None))
        step[:3, 3] = floats(o.get("xyz") if o is not None else None)
        t = t @ step
        if j.get("type") in ("revolute", "continuous"):
            idx = int(re.search(r"(\d+)$", j.get("name")).group(1)) - 1
            rot = np.eye(4)
            rot[:3, :3] = axis_angle(floats(j.find("axis").get("xyz"), "1 0 0"), q[idx])
            t = t @ rot
    return t


def pose(urdf_root, q):
    t = fk(urdf_root, q)
    return [*t[:3, 3], *matrix_rpy(t[:3, :3])]


def fmt(p):
    return "[" + ", ".join(f"{v:.6f}" for v in p) + "]"


def main():
    urdf, seq = sys.argv[1], sys.argv[2]
    write = "--write" in sys.argv
    root = ET.parse(urdf).getroot()
    print("zero pose tip:", fmt(pose(root, [0.0] * 6)))

    lines = open(seq).read().split("\n")
    pending = None
    for i, line in enumerate(lines):
        m = re.match(r"\s*#\s*from joints\s+(.+)$", line)
        if m:
            pending = [float(v) for v in m.group(1).replace(",", " ").split()]
            continue
        m = re.match(r"(\s*pose:\s*)\[.*\](.*)$", line)
        if m and pending is not None:
            new = fmt(pose(root, pending))
            print(f"joints {pending} -> {new}")
            lines[i] = m.group(1) + new + m.group(2)
            pending = None
    if write:
        open(seq, "w").write("\n".join(lines))
        print("written:", seq)


if __name__ == "__main__":
    main()
