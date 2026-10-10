#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Verify PCD binary integrity: header POINTS vs actual file size.
用法: python3 check_pcd.py [目录，默认 FAST-LIVO2/Log/pcd]
"""
import os, sys

def parse_header(path):
    with open(path, 'rb') as f:
        raw = f.read(2048)
    idx = raw.find(b'DATA')
    if idx < 0:
        return None, None
    end = raw.find(b'\n', idx)
    if end < 0:
        end = len(raw)
    header = raw[:end+1].decode('ascii', errors='replace')
    points = None
    sizes = None
    data_type = None
    for line in header.splitlines():
        if line.startswith('POINTS'):
            points = int(line.split()[1])
        elif line.startswith('SIZE'):
            sizes = [int(x) for x in line.split()[1:]]
        elif line.startswith('DATA'):
            data_type = line.split()[1]
    return (points, sizes, data_type, end+1), header

def check(path):
    info, header = parse_header(path)
    if info is None or info[0] is None:
        print(f'[SKIP] {os.path.basename(path)}: 无法解析头部')
        return None
    points, sizes, data_type, header_len = info
    if data_type != 'binary':
        print(f'[SKIP] {os.path.basename(path)}: 非二进制PCD ({data_type})')
        return None
    psize = sum(sizes)
    actual = os.path.getsize(path)
    expected = header_len + points * psize
    diff = actual - expected
    ok = (diff == 0)
    status = '✅ 完整' if ok else f'❌ 损坏 (差 {diff} 字节)'
    print(f'{status} {os.path.basename(path)}: 声明 {points} 点 × {psize}B + 头 {header_len}B')
    print(f'   理论 {expected} B vs 实际 {actual} B')
    return ok

if __name__ == '__main__':
    d = sys.argv[1] if len(sys.argv) > 1 else '/home/fouruav/catkin_ws/src/FAST-LIVO2/Log/pcd'
    results = []
    for name in sorted(os.listdir(d)):
        if name.endswith('.pcd') and name != '.gitkeep':
            r = check(os.path.join(d, name))
            if r is not None:
                results.append(r)
    print()
    if results and all(results):
        print('== 结论: 全部完整 ✅')
    elif any(results):
        print('== 结论: 存在损坏文件 ❌ 请检查')
    else:
        print('== 结论: 无 PCD 文件可检查')
