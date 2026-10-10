#!/bin/bash
# ============================================
# 🔐 安全打包脚本：防止部署时代码泄露
# ============================================
# 用法:
#   bash build_secure.sh [输出目录]
#     默认输出到 <项目目录>/deploy/
#
# 原理:
#   1. 在临时目录复制源码
#   2. 用 Cython 将 .py 编译为二进制扩展 .so
#   3. 删除源码 .py 与中间产物 .c (防泄露)
#   4. 为每个入口生成薄启动器 *_launcher.py
#   5. 生成与 start_qgc_servers.sh 兼容的 *2.py 包装器
#   6. 拷贝资源文件 (yaml/sh/desktop/README 等) 并修正入口调用
#
# 原工程目录保持完整源码，不受影响。部署目录中不含任何 .py 源码。
# ============================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

OUTPUT="${1:-deploy}"
OUTPUT_DIR="$SCRIPT_DIR/$OUTPUT"
WORK="$(mktemp -d /tmp/nx_secure_build.XXXXXX)"
trap 'rm -rf "$WORK"' EXIT

# 入口映射: 模块名 -> 入口函数 (用于生成 *_launcher.py)
declare -A ENTRY_FUNC=(
  [nx_command_server]=start_command_server
  [nx_touch_server]=start_touch_server
  [rviz_full_stream]=run_full_stream
  [odom_to_mavros]=run_odom_to_mavros
)

# 与 start_qgc_servers.sh 兼容的 *2.py 包装器映射: 包装器名 -> 模块:入口函数
declare -A LEGACY2=(
  [nx_command_server2]=nx_command_server:start_command_server
  [nx_touch_server2]=nx_touch_server:start_touch_server
  [rviz_full_stream2]=rviz_full_stream:run_full_stream
)

# 需要拷贝到部署目录的资源文件
RESOURCES=(
  Radar_Launcher.yaml
  User_define_commands.yaml
  start_qgc_servers.sh
  start_fastlivo2.sh
  odin_stream.desktop
  qgc_servers.desktop
  依赖.txt
  README.md
  .gitignore
)

# ---------- 0. 依赖检查 ----------
command -v python3 >/dev/null 2>&1 || { echo "❌ 未找到 python3"; exit 1; }
python3 -c "import Cython" 2>/dev/null || { echo "❌ 缺少 Cython，请先安装: pip3 install cython"; exit 1; }
command -v gcc >/dev/null 2>&1 || { echo "❌ 缺少 gcc 编译器，请先安装: sudo apt install gcc"; exit 1; }

echo "🔐 开始安全打包 -> $OUTPUT_DIR"

# ---------- 1. 复制源码到临时构建目录 ----------
cp *.py "$WORK"/
echo "  [1/6] 复制源码到临时目录"

# ---------- 2. Cython 编译 .py -> .so ----------
echo "  [2/6] Cython 编译 .py -> .so (请稍候)..."
(cd "$WORK" && python3 setup.py build_ext --inplace >/dev/null)

# ---------- 3. 删除源码与中间产物 ----------
rm -f "$WORK"/*.py "$WORK"/*.c
rm -rf "$WORK"/build
echo "  [3/6] 已删除源码(.py)与中间产物(.c)"

# ---------- 4. 准备输出目录 ----------
rm -rf "$OUTPUT_DIR"
mkdir -p "$OUTPUT_DIR"

# ---------- 5. 拷贝编译产物 .so ----------
cp "$WORK"/*.so "$OUTPUT_DIR"/
echo "  [4/6] 拷贝编译产物 .so"

# ---------- 6. 生成薄启动器 ----------
gen_launcher() {
    local mod="$1" func="$2" path="$3"
    cat > "$path" <<EOF
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 自动生成启动器：核心逻辑已编译为 Cython 扩展(.so)，源码不随部署分发
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ${mod}
${mod}.${func}()
EOF
    chmod +x "$path"
}

for mod in "${!ENTRY_FUNC[@]}"; do
    gen_launcher "$mod" "${ENTRY_FUNC[$mod]}" "$OUTPUT_DIR/${mod}_launcher.py"
done

for legacy in "${!LEGACY2[@]}"; do
    mod="${LEGACY2[$legacy]%%:*}"
    func="${LEGACY2[$legacy]##*:}"
    cat > "$OUTPUT_DIR/$legacy.py" <<EOF
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 部署版包装器：import 已编译的 Cython 扩展，与 start_qgc_servers.sh 调用兼容
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ${mod} import ${func}

if __name__ == '__main__':
    ${func}()
EOF
done
echo "  [5/6] 生成启动器(*_launcher.py)与兼容包装器(*2.py)"

# ---------- 7. 拷贝资源文件并修正入口调用 ----------
for res in "${RESOURCES[@]}"; do
    [ -f "$res" ] && cp "$res" "$OUTPUT_DIR"/
done

# 修正拷贝的 .desktop / .sh 中入口调用: 引用 .py 源码改为引用 _launcher.py
find "$OUTPUT_DIR" -maxdepth 1 \( -name '*.desktop' -o -name '*.sh' \) -print0 | while IFS= read -r -d '' f; do
    sed -i -E \
        -e 's|/nx_command_server\.py|/nx_command_server_launcher.py|g' \
        -e 's|/rviz_full_stream\.py|/rviz_full_stream_launcher.py|g' \
        -e 's|/nx_touch_server\.py|/nx_touch_server_launcher.py|g' \
        -e 's|/odom_to_mavros\.py|/odom_to_mavros_launcher.py|g' \
        "$f"
done
echo "  [6/6] 拷贝资源文件并修正入口调用"

# ---------- 8. 生成部署说明 ----------
cat > "$OUTPUT_DIR/DEPLOY_README.txt" <<'EOF'
============================================
 NX_code 安全部署包 - 使用说明
============================================

本目录由 build_secure.sh 自动生成。
核心代码已编译为 Cython 扩展(.so)，不含任何 .py 源码，防止代码泄露。

一、部署步骤
  1. 将本目录整体拷贝到目标 NX 机（路径可自定，建议放在
     ~/catkin_ws/src/task_run_scripts/scripts/NX_code/）
  2. 确认目标机已安装依赖（见 依赖.txt）与 Cython 编译产物所需运行库
  3. 修改配置: Radar_Launcher.yaml / User_define_commands.yaml
     （若部署路径与源机不同，请同步修改 start_qgc_servers.sh、
       *.desktop 与 Radar_Launcher.yaml 中的绝对路径）

二、启动方式（与源码版一致）
  - 开机自启: 将 qgc_servers.desktop、odin_stream.desktop 放入
    ~/.config/autostart/（内部已自动改为调用 *_launcher.py）
  - 手动启动: bash start_qgc_servers.sh

三、服务启动器对照
  python3 nx_command_server_launcher.py   # 指令+监测服务 (6003/6004)
  python3 nx_touch_server_launcher.py     # 触控转鼠标 (6002)
  python3 rviz_full_stream_launcher.py --worker  # 全屏推流 (6000，含心跳)
  python3 odom_to_mavros_launcher.py      # 里程计转发

  (start_qgc_servers.sh 调用的 *2.py 为部署版包装器，import .so 扩展)

四、重新打包
  回到源码工程目录运行: bash build_secure.sh [新目录名]

五、安全说明
  .so 为编译产物，可从反编译层面提高保护强度，但无法做到绝对不可逆。
  请妥善保管源码工程与 Git 仓库权限。
EOF

echo ""
echo "✅ 打包完成！部署目录: $OUTPUT_DIR"
echo "   部署时拷贝整个目录到目标 NX 即可，详见 DEPLOY_README.txt"
