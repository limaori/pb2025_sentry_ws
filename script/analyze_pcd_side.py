#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Side-view + Z histogram diagnostics for the SLAM point cloud map."""
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import os

PCD = sys.argv[1] if len(sys.argv) > 1 else \
    "/home/srm/pb2025_sentry_ws/src/pb2025_sentry_nav/point_lio/PCD/scans.pcd"
OUT = sys.argv[2] if len(sys.argv) > 2 else "/home/srm/pb2025_sentry_ws/log_diag"

def read_pcd(path):
    with open(path, "rb") as f:
        header = b""
        while True:
            line = f.readline()
            header += line
            if line.startswith(b"DATA"):
                break
        fields = None
        for line in header.decode("ascii", "replace").splitlines():
            if line.startswith("FIELDS"):
                fields = line.split()[1:]
        raw = np.fromfile(f, dtype=np.float32)
        npts = int(len(raw) / len(fields))
        return fields, raw.reshape(npts, len(fields))

fields, data = read_pcd(PCD)
c = {n: i for i, n in enumerate(fields)}
x, y, z = data[:, c["x"]], data[:, c["y"]], data[:, c["z"]]

os.makedirs(OUT, exist_ok=True)

# histogram of Z
fig, ax = plt.subplots(figsize=(12, 6))
ax.hist(z, bins=200, color="teal")
ax.set_xlabel("Z (m)")
ax.set_ylabel("point count")
ax.set_title("Z-height histogram (flat floor should be a narrow peak near 0)")
fig.tight_layout()
zh_path = os.path.join(OUT, "map_z_histogram.png")
fig.savefig(zh_path, dpi=110)
plt.close(fig)
print("saved", zh_path)

# quantify vertical spread
for lo, hi in [(-2, 3), (3, 6), (6, 12), (12, 30), (-12, -2)]:
    frac = np.mean((z >= lo) & (z < hi))
    print(f"Z in [{lo:>3},{hi:>3}): {frac*100:6.2f}%")

# Y-Z side projection (viewed from X axis)
fig, ax = plt.subplots(figsize=(12, 8))
ax.scatter(y, z, s=0.4, marker=".", c="steelblue", linewidths=0)
ax.set_xlabel("Y (m)")
ax.set_ylabel("Z (m)")
ax.set_title("Side view (Y-Z projection); floor should be a thin horizontal line near Z=0")
ax.invert_yaxis()  # no; keep normal
fig.tight_layout()
yz_path = os.path.join(OUT, "map_side_YZ.png")
fig.savefig(yz_path, dpi=110)
plt.close(fig)
print("saved", yz_path)

# X-Z side projection
fig, ax = plt.subplots(figsize=(12, 8))
ax.scatter(x, z, s=0.4, marker=".", c="indianred", linewidths=0)
ax.set_xlabel("X (m)")
ax.set_ylabel("Z (m)")
ax.set_title("Side view (X-Z projection)")
fig.tight_layout()
xz_path = os.path.join(OUT, "map_side_XZ.png")
fig.savefig(xz_path, dpi=110)
plt.close(fig)
print("saved", xz_path)

# A "floor" fit: robust histogram peak of Z
hist, edges = np.histogram(z, bins=200)
peak = edges[np.argmax(hist)]
print("\nmost common Z (mode): %.3f m" % peak)
print("median Z: %.3f m" % np.median(z))
print("DONE")
