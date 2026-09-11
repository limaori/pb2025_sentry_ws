#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""检查先验点云 PCD 与栅格地图 pgm 是否在同一坐标系，并给出最佳 yaw。

为什么需要它:
  small_gicp 重定位用的 prior_pcd_file 必须与 map_server 加载的栅格图在同一
  坐标系，否则重定位会收敛到完全错误的地图位姿（而且不一定报错）。
  Point-LIO 直接落盘的 PCD 属于 camera_init 系；如果这份 PCD 是离线回放 bag
  建的图，它的 camera_init 与当时在线建图得到的地图系可能差一个任意 yaw。

原理:
  把 PCD 俯视投影成栅格，按 1 度步长旋转，每一步用 FFT 互相关找最佳平移，
  取与 pgm 占据栅格重合度最高的 yaw。yaw≈0 说明 PCD 已经在地图系。

用法:
  python3 script/check_pcd_map_alignment.py [pgm] [pcd ...]
  默认对比 maps/srm_site_01.pgm 与 maps/ 下的两份 srm_site_01 点云。
"""
import sys
import numpy as np
from scipy.ndimage import rotate as ndrotate
from scipy.signal import fftconvolve

WS = "/home/srm/pb2025_sentry_ws"
DEFAULT_PGM = f"{WS}/maps/srm_site_01.pgm"
DEFAULT_PCDS = [
    f"{WS}/maps/srm_site_01.pcd",
    f"{WS}/maps/srm_site_01_mapframe.pcd",
]
RES = 0.05  # 与 map/reality/srm_site_01.yaml 的 resolution 一致


def read_pgm(path):
    """返回 (占据掩码, 宽, 高)。行 0 = 图像顶部 = 地图 y 最大处。"""
    data = open(path, "rb").read()
    tokens, i = [], 0
    while len(tokens) < 4:
        while data[i : i + 1].isspace():
            i += 1
        if data[i : i + 1] == b"#":
            while data[i : i + 1] not in (b"\n", b""):
                i += 1
            continue
        j = i
        while not data[j : j + 1].isspace():
            j += 1
        tokens.append(data[i:j])
        i = j
    width, height = int(tokens[1]), int(tokens[2])
    i += 1
    img = np.frombuffer(data[i : i + width * height], dtype=np.uint8)
    img = img.reshape(height, width)
    # map_server: 像素低于 occupied_thresh(0.65) 视为占据
    return img < int(0.65 * 255), width, height


def read_pcd_xy(path):
    with open(path, "rb") as f:
        header = b""
        while True:
            line = f.readline()
            header += line
            if line.startswith(b"DATA"):
                break
        fields, n = [], 0
        for ln in header.decode("ascii", "replace").splitlines():
            if ln.startswith("FIELDS"):
                fields = ln.split()[1:]
            elif ln.startswith("POINTS"):
                n = int(ln.split()[1])
        raw = f.read(n * len(fields) * 4)
    arr = np.frombuffer(raw, dtype=np.float32).reshape(-1, len(fields))
    return arr[:, fields.index("x")], arr[:, fields.index("y")]


def rasterise(x, y, res, pad=2.0):
    xmin, xmax = x.min() - pad, x.max() + pad
    ymin, ymax = y.min() - pad, y.max() + pad
    w = int(np.ceil((xmax - xmin) / res))
    h = int(np.ceil((ymax - ymin) / res))
    ix = np.clip(((x - xmin) / res).astype(np.int32), 0, w - 1)
    iy = np.clip(((y - ymin) / res).astype(np.int32), 0, h - 1)
    img = np.zeros((h, w), dtype=np.float32)
    np.add.at(img, (iy, ix), 1.0)
    return img[::-1, :]  # 行 0 = 顶部, 与 pgm 一致


def score_yaw(pcd_img, target, yaw_deg):
    rot = (ndrotate(pcd_img, yaw_deg, reshape=True, order=1, cval=0.0) > 0).astype(np.float32)
    corr = fftconvolve(target, rot[::-1, ::-1], mode="full")
    return corr.max() / max(target.sum(), 1)


def main():
    pgm = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PGM
    pcds = sys.argv[2:] or DEFAULT_PCDS
    occ, pw, ph = read_pgm(pgm)
    print(f"栅格图 {pgm}: {pw}x{ph}, 占据格 {int(occ.sum())}, 分辨率 {RES} m")
    target = occ.astype(np.float32)
    for path in pcds:
        x, y = read_pcd_xy(path)
        img = rasterise(x, y, RES)
        results = sorted(
            ((score_yaw(img, target, yaw), yaw) for yaw in range(0, 360, 2)),
            reverse=True,
        )
        print(f"\n{path}")
        print(f"  {len(x)} 点, bbox x[{x.min():.2f},{x.max():.2f}] y[{y.min():.2f},{y.max():.2f}]")
        print("  top-3 yaw 与重合度:")
        for score, yaw in results[:3]:
            print(f"    yaw={yaw:6.1f}  重合度={score:.4f}")
        best_yaw = results[0][1]
        if best_yaw <= 4 or best_yaw >= 356:
            print("  => 已在地图系, 可直接用作 prior_pcd_file")
        else:
            print(f"  => 不在地图系, 大概需要绕 Z 转 {best_yaw} 度, 直接用于重定位会定位错")


if __name__ == "__main__":
    main()
