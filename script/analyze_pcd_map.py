#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Parse a binary PCD (x y z intensity nx ny nz curvature) and render
top-down / intensity views plus summary stats, so we can judge whether a
point-cloud SLAM map looks sane."""

import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import Normalize
import os

PCD = sys.argv[1] if len(sys.argv) > 1 else \
    "/home/srm/pb2025_sentry_ws/src/pb2025_sentry_nav/point_lio/PCD/scans.pcd"
OUT = sys.argv[2] if len(sys.argv) > 2 else \
    "/home/srm/pb2025_sentry_ws/log_diag"

def read_pcd(path):
    with open(path, "rb") as f:
        header = b""
        while True:
            line = f.readline()
            header += line
            if line.startswith(b"DATA"):
                break
        # parse header
        fields = []
        nfields = 0
        for line in header.decode("ascii", "replace").splitlines():
            if line.startswith("FIELDS"):
                fields = line.split()[1:]
            elif line.startswith("POINTS") or line.startswith("WIDTH"):
                try:
                    nfields = int(line.split()[1])
                except ValueError:
                    pass
        data_type = None
        for line in header.decode("ascii", "replace").splitlines():
            if line.startswith("TYPE"):
                types = line.split()[1:]
            if line.startswith("DATA"):
                data_type = line.split()[1]
        if data_type != "binary":
            raise RuntimeError(f"Expected binary PCD, got {data_type}")
        raw = np.fromfile(f, dtype=np.float32)
    npts = int(len(raw) / len(fields))
    arr = raw.reshape(npts, len(fields))
    return header, fields, arr

header, fields, data = read_pcd(PCD)
print("FIELDS:", fields)
print("N points:", data.shape[0])

# field indices
name2col = {name: i for i, name in enumerate(fields)}
x = data[:, name2col["x"]]
y = data[:, name2col["y"]]
z = data[:, name2col["z"]]
inten = data[:, name2col["intensity"]]

print("\n=== Statistics ===")
for col, label in [("x", "X"), ("y", "Y"), ("z", "Z")]:
    v = data[:, name2col[col]]
    print(f"{label}: min={v.min():.3f}  max={v.max():.3f}  span={v.max()-v.min():.3f}")
print("intensity: min=%.3f max=%.3f mean=%.3f" % (inten.min(), inten.max(), inten.mean()))

# non-finite check
bad = ~np.isfinite(data).all(axis=1)
print("non-finite rows:", int(bad.sum()))

# density estimate: count of points in a coarse grid
print("number of points: %d (%.2f M)" % (data.shape[0], data.shape[0] / 1e6))

# Downsample for rendering
N = data.shape[0]
sample_n = 700000
step = max(1, N // sample_n)
idx = np.arange(0, N, step)
xs, ys, zs, ins = x[idx], y[idx], z[idx], inten[idx]
print("rendering with %d samples" % len(idx))

os.makedirs(OUT, exist_ok=True)

# ---- Top-down view colored by height ----
fig, ax = plt.subplots(figsize=(14, 10))
sc = ax.scatter(xs, ys, c=zs, s=0.5, marker=".", cmap="viridis", linewidths=0)
ax.set_aspect("equal")
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_title("Top-down (XY) colored by Z height")
plt.colorbar(sc, ax=ax, label="Z (m)")
fig.tight_layout()
top_path = os.path.join(OUT, "map_top_view.png")
fig.savefig(top_path, dpi=110)
plt.close(fig)
print("saved", top_path)

# ---- Top-down view colored by intensity ----
fig, ax = plt.subplots(figsize=(14, 10))
sc = ax.scatter(xs, ys, c=ins, s=0.5, marker=".", cmap="magma", linewidths=0)
ax.set_aspect("equal")
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_title("Top-down (XY) colored by intensity")
plt.colorbar(sc, ax=ax, label="intensity")
fig.tight_layout()
int_path = os.path.join(OUT, "map_intensity_view.png")
fig.savefig(int_path, dpi=110)
plt.close(fig)
print("saved", int_path)

# ---- 3/4 perspective view ----
fig = plt.figure(figsize=(14, 10))
ax = fig.add_subplot(111, projection="3d")
sc = ax.scatter(xs, ys, zs, c=zs, s=0.3, cmap="viridis", linewidths=0)
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_zlabel("Z (m)")
ax.set_title("Perspective 3D view")
# set equal aspect ratios roughly
ax.set_box_aspect((1, 1, 0.35))
fig.tight_layout()
persp_path = os.path.join(OUT, "map_perspective_view.png")
fig.savefig(persp_path, dpi=100)
plt.close(fig)
print("saved", persp_path)

print("\nDONE")
