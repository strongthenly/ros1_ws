#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PCD 着色转换工具：把 XYZI 点云转成 XYZRGB（按高度 Z 渐变着色）
用法: python3 colorize_pcd.py <输入.pcd> [输出.pcd]
"""
import struct
import sys
import os

def read_pcd_header(f):
    header = []
    while True:
        line = f.readline()
        if not line:
            break
        header.append(line)
        if line.startswith(b'DATA'):
            break
    return header

def colorize_jet(t):
    t = max(0.0, min(1.0, t))
    r = max(0.0, min(1.0, 1.5 - abs(4.0 * t - 3.0)))
    g = max(0.0, min(1.0, 1.5 - abs(4.0 * t - 2.0)))
    b = max(0.0, min(1.0, 1.5 - abs(4.0 * t - 1.0)))
    return int(r * 255), int(g * 255), int(b * 255)

def convert(in_path, out_path=None):
    if out_path is None:
        base, ext = os.path.splitext(in_path)
        out_path = f"{base}_color.pcd"

    with open(in_path, 'rb') as f:
        header = read_pcd_header(f)
        points = 0
        for line in header:
            if line.startswith(b'POINTS'):
                points = int(line.decode().split()[1])
        data_start = f.tell()

    point_size = 32  # x y z intensity normal_x normal_y normal_z curvature
    print(f"点数: {points}, 每点 {point_size} 字节")

    # 第一遍：找 Z 范围
    z_min, z_max = float('inf'), float('-inf')
    with open(in_path, 'rb') as f:
        f.seek(data_start)
        for i in range(points):
            data = f.read(point_size)
            if len(data) < point_size:
                break
            x, y, z = struct.unpack('<fff', data[:12])
            if z < z_min: z_min = z
            if z > z_max: z_max = z
    z_range = z_max - z_min
    if z_range < 1e-6:
        z_range = 1.0
    print(f"Z 范围: {z_min:.2f} ~ {z_max:.2f}")

    # 第二遍：着色 + 写新文件
    with open(in_path, 'rb') as f, open(out_path, 'wb') as out:
        new_header = (
            b"# .PCD v0.7 - Point Cloud Data file format\n"
            b"VERSION 0.7\n"
            b"FIELDS x y z rgb\n"
            b"SIZE 4 4 4 4\n"
            b"TYPE F F F F\n"
            b"COUNT 1 1 1 1\n"
            + ("WIDTH %d\n" % points).encode()
            + b"HEIGHT 1\n"
            + b"VIEWPOINT 0 0 0 1 0 0 0\n"
            + ("POINTS %d\n" % points).encode()
            + b"DATA binary\n"
        )
        out.write(new_header)

        f.seek(data_start)
        for i in range(points):
            data = f.read(point_size)
            if len(data) < point_size:
                break
            x, y, z = struct.unpack('<fff', data[:12])
            t = (z - z_min) / z_range
            r, g, b = colorize_jet(t)
            rgb = (r << 16) | (g << 8) | b
            out.write(struct.pack('<fffI', x, y, z, rgb))

    print(f"✅ 已生成: {out_path} (按高度 Z 着色, jet 色图)")
    return out_path

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("用法: python3 colorize_pcd.py <输入.pcd> [输出.pcd]")
        sys.exit(1)
    convert(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else None)
