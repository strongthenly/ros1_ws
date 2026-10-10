#!/bin/bash
# ============================================================
#  组装实机导出包（在 WSL ros1-twin 里跑）
#  来源:
#    A) ~/fuel_ws/src/FUEL/fuel_planner        探索算法（排除 .bak / bag / 日志）
#    B) ~/fuel_ws/src/fuel_bridge              桥接层（源码原样，仅取 .py/.launch）
#    C) /mnt/c/Users/liyan/Desktop/fouruav_fuel_export/   实机版 launch + 脚本（Windows 侧写好的）
#  产出: ~/fuel_real_pkg/  +  ~/fuel_real_pkg.tar.gz
# ============================================================
set -u
SRC_FUEL=/home/liyan/fuel_ws/src/FUEL/fuel_planner
SRC_BR=/home/liyan/fuel_ws/src/fuel_bridge
WIN=/mnt/c/Users/liyan/Desktop/fouruav_fuel_export
PKG=/home/liyan/fuel_real_pkg
OUT=/home/liyan/fuel_real_pkg.tar.gz

echo "===== 0. 清理旧包 ====="
rm -rf "$PKG" "$OUT"
mkdir -p "$PKG/src" "$PKG/launch" "$PKG/scripts" "$PKG/config"

echo "===== 1. 拷 fuel_planner（排除 .bak / bag / 日志）====="
cd "$SRC_FUEL" || exit 1
find . -type d \( -name bag -o -name .git -o -name __pycache__ \) -prune -o \
     -type f ! -name '*.bak*' ! -name '*.orig*' ! -name '*.log' ! -name 'log.md' ! -name 'test.txt' \
     -print0 | while IFS= read -r -d '' f; do
  mkdir -p "$PKG/src/fuel_planner/$(dirname "$f")"
  cp -p "$f" "$PKG/src/fuel_planner/$f"
done
echo "   fuel_planner: $(find "$PKG/src/fuel_planner" -type f | wc -l) 个文件"

echo "===== 2. 拷 fuel_bridge（源码 + launch）====="
cd "$SRC_BR" || exit 1
find . -type f ! -name '*.bak*' ! -name '*.orig*' ! -name '*.log' \
     -print0 | while IFS= read -r -d '' f; do
  mkdir -p "$PKG/src/fuel_bridge/$(dirname "$f")"
  cp -p "$f" "$PKG/src/fuel_bridge/$f"
done
echo "   fuel_bridge: $(find "$PKG/src/fuel_bridge" -type f | wc -l) 个文件"

echo "===== 3. 放进实机版 launch / 脚本（来自 Windows 侧）====="
cp -p "$WIN/launch/fuel_real_airborne.launch" "$PKG/launch/"
cp -p "$WIN/scripts/"*.sh "$PKG/scripts/"
cp -p "$WIN/UserCmd_FUEL_append.yaml" "$PKG/" 2>/dev/null || true
chmod +x "$PKG/scripts/"*.sh
echo "   launch: $(ls "$PKG/launch" | tr '\n' ' ')"
echo "   scripts: $(ls "$PKG/scripts" | tr '\n' ' ')"

echo "===== 4. 清单 + MD5（用于上机后逐项核对）====="
{
  echo "# fuel_real_pkg MANIFEST"
  echo "生成时间: $(date -Iseconds)"
  echo "来源机: ros1-twin (WSL)  ->  目标: fouruav (Orin NX)"
  echo ""
  echo "## 关键文件 MD5（上机后 pscp 传完请复核这几个）"
  for f in "$PKG/src/fuel_bridge/scripts/fuel_input_bridge.py" \
           "$PKG/src/fuel_bridge/scripts/fuel_ctrl_bridge.py" \
           "$PKG/src/fuel_bridge/scripts/goal_arbiter.py" \
           "$PKG/launch/fuel_real_airborne.launch" \
           "$PKG/scripts/fuel_shadow_real.sh" \
           "$PKG/scripts/fuel_stop_real.sh"; do
    [ -f "$f" ] && echo "$(md5sum "$f" | cut -d' ' -f1)  ${f#$PKG/}"
  done
  echo ""
  echo "## 包内文件数"
  echo "total: $(find "$PKG" -type f | wc -l)"
  echo "fuel_planner: $(find "$PKG/src/fuel_planner" -type f | wc -l)"
  echo "fuel_bridge: $(find "$PKG/src/fuel_bridge" -type f | wc -l)"
  echo ""
  echo "## 与仿真版的关系"
  echo "fuel_bridge 的 .py 与 WSL 仿真版【逐字节相同】（本脚本直接 cp，未改）"
  echo "唯一新增：launch/fuel_real_airborne.launch（实机适配版）+ scripts/*_real.sh"
} > "$PKG/MANIFEST.md"

echo "===== 5. 打 tar.gz ====="
cd /home/liyan && tar czf "$OUT" fuel_real_pkg
ls -lh "$OUT"
echo ""
echo "===== 完成 ====="
echo "包目录: $PKG"
echo "压缩包: $OUT"
echo "清单  : $PKG/MANIFEST.md"
