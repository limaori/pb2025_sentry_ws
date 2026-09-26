#!/usr/bin/env python3
# =============================================================
# 清理栅格地图里的"孤立小黑点"(噪点)
#
# 为什么需要:
#   建图/修图后常留下几个格子的孤立占用点(0.1x0.1 m 量级)。它们会让
#   StaticLayer 把那格标成致命障碍(254), 再叠加 robot_radius 的内切半径
#   (内切半径内的格子代价=253), 于是:
#     - Theta*  : "Either of the start or goal pose are an obstacle!" -> 不规划
#     - 控制器  : 采样的路径点撞到 >=253 -> "Collision detected" -> 不发速度
#   结果就是车原地不动、所有目标都失败 —— 而雷达其实什么都没看到。
#   (2026-09-16 现场就是这么卡住的: 227_0916.pgm 里 4 个格的黑点离车 0.15 m)
#
# 判据:
#   占用连通块 面积 <= --max-cells 且 距离其它占用格 >= --min-gap 米(默认孤立)
#   -> 认定为噪点。默认只报告, 加 --clean 才改图(会先备份原图)。
#
# 用法:
#   python3 script/clean_map_specks.py maps/227_0916/227_0916.pgm            # 只报告
#   python3 script/clean_map_specks.py maps/227_0916/227_0916.pgm --clean    # 清理(备份)
#   python3 script/clean_map_specks.py maps/227_0916/227_0916.yaml           # 也可传 yaml
# =============================================================
import argparse
import os
import shutil
import sys
import time

import numpy as np
import yaml
from PIL import Image
from scipy import ndimage

FREE_PX = 254          # 写回"自由"用的像素值(mode: trinary 下 254 -> free)
OCC_MAX_PX = 5         # 像素值 <= 这个数视为"占用/黑"


def load_pgm(path):
    im = Image.open(path)
    return im, np.array(im)


def resolve(arg):
    if arg.endswith(('.yaml', '.yml')):
        meta = yaml.safe_load(open(arg, encoding='utf-8'))
        img = meta.get('image')
        if not img:
            sys.exit(f'[错误] {arg} 里没有 image 字段')
        p = img if os.path.isabs(img) else os.path.join(os.path.dirname(arg), img)
        return p, meta
    return arg, None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('map', help='pgm 或 yaml 路径')
    ap.add_argument('--max-cells', type=int, default=20,
                    help='占用连通块最大格数, 小于等于它才算噪点(默认 20 格 = 0.05 m^2)')
    ap.add_argument('--min-gap', type=float, default=0.30,
                    help='与其它占用格的最小间距(米), 默认 0.30(保证是"孤零零浮在空地里的")')
    ap.add_argument('--clean', action='store_true', help='真的改图(会备份成 .bak_时间戳)')
    ap.add_argument('--free-value', type=int, default=FREE_PX,
                    help=f'噪点改成什么像素值(默认 {FREE_PX} = free)')
    args = ap.parse_args()

    pgm, meta = resolve(args.map)
    if not os.path.isfile(pgm):
        sys.exit(f'[错误] 找不到 {pgm}')
    res = float(meta['resolution']) if meta else 0.05
    ox = oy = 0.0
    if meta and meta.get('origin'):
        ox, oy = float(meta['origin'][0]), float(meta['origin'][1])

    im, a = load_pgm(pgm)
    h, w = a.shape
    occ = a <= OCC_MAX_PX
    lab, n = ndimage.label(occ, structure=np.ones((3, 3), int))
    print(f'地图 {pgm}: {w}x{h} @ {res} m, 占用连通块 {n} 个')

    # 每个占用块到"其它占用块"的距离
    dist_to_other = {}
    specks = []
    for i in range(1, n + 1):
        m = lab == i
        size = int(m.sum())
        if size > args.max_cells:
            continue
        other = occ & ~m
        d = ndimage.distance_transform_edt(~other) * res
        gap = float(d[m].min())
        ys, xs = np.where(m)
        cx = ox + xs.mean() * res
        cy = oy + ((h - 1) - ys.mean()) * res
        item = dict(size=size, gap=gap, cx=cx, cy=cy,
                    bbox=(int(xs.min()), int(xs.max()), int(ys.min()), int(ys.max())))
        dist_to_other[i] = d
        if gap >= args.min_gap:
            specks.append(item)

    if not specks:
        print(f'\n没有发现"<= {args.max_cells} 格 且 离别的占用格 >= {args.min_gap} m"的孤立噪点。')
        return 0

    print(f'\n发现 {len(specks)} 个孤立噪点 (<= {args.max_cells} 格, '
          f'离其它占用格 >= {args.min_gap} m):')
    for k, s in enumerate(specks, 1):
        print(f'  {k}. {s["size"]:3d} 格 ({s["size"]*res*res:.3f} m^2)  '
              f'map({s["cx"]:+.2f},{s["cy"]:+.2f})  离最近占用格 {s["gap"]:.2f} m  '
              f'像素范围 x[{s["bbox"][0]}..{s["bbox"][1]}] y[{s["bbox"][2]}..{s["bbox"][3]}]')

    if not args.clean:
        print('\n[dry-run] 未修改文件。确认无误后加 --clean 执行清理'
              '(会先备份成 <名字>.pgm.bak_<时间戳>)。')
        return 0

    out = a.copy()
    for s in specks:
        x0, x1, y0, y1 = s['bbox']
        sub = lab[y0:y1 + 1, x0:x1 + 1]
        ids = set(np.unique(sub[sub > 0]).tolist())
        for i in ids:
            m = (lab == i) if i in dist_to_other or True else None
            out[lab == i] = args.free_value
    bak = f'{pgm}.bak_{time.strftime("%Y%m%d_%H%M%S")}'
    shutil.copy2(pgm, bak)
    Image.fromarray(out.astype(np.uint8), mode='L').save(pgm)
    print(f'\n已清理 {len(specks)} 个噪点并写回 {pgm}')
    print(f'原图备份: {bak}')
    print('注意: map_server 是启动时读图的, 要让正在跑的导航生效, 需要重启导航栈'
          '(或调用 /map_server/load_map)。')
    return 0


if __name__ == '__main__':
    sys.exit(main())
