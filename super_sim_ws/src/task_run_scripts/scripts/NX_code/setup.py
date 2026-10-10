#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Cython 编译脚本：将核心 Python 源码编译为扩展模块(.so)
防止工程部署时源码泄露。

用法:
    python3 setup.py build_ext --inplace     # 在当前目录生成 .so
    bash build_secure.sh                     # 一键安全打包(推荐)

说明:
    - 编译产物为 *.cpython-*-x86_64-linux-gnu.so，Python 可直接 import
    - 排除备份版本(*2.py)、启动器(_launcher.py)与本脚本
"""
from setuptools import setup
from Cython.Build import cythonize
import os

# 不参与编译的文件
EXCLUDE = {
    "setup.py",
}

files = [
    f for f in os.listdir(".")
    if f.endswith(".py")
    and f not in EXCLUDE
    and not f.endswith("_launcher.py")
    and not f.endswith("2.py")          # 排除 *2.py 备份/包装版本
]

setup(
    ext_modules=cythonize(
        files,
        compiler_directives={"language_level": "3"},
        quiet=True,
    )
)
