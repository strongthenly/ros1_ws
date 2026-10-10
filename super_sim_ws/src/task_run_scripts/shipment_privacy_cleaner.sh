#!/usr/bin/env bash
# =============================================================================
#  发货前隐私清理脚本  shipment_privacy_cleaner.sh
#  适用系统: Ubuntu (aarch64 / Jetson) · ROS catkin 工作区
#
#  设计原则:
#    1. 默认只「扫描」不删除 —— 先看报告，再决定执行。
#    2. 只清隐私「痕迹」(历史/缓存/日志/凭据/浏览器登录态)，
#       保留全部软件与源码(catkin_ws / PX4 / NX_code / 驱动 .so 等)。
#    2.1 网络连接(WiFi 密码 / 有线静态 IP)一律保留，不删除。
#    3. 每次清理前把「将删除清单」写入审计文件，可回溯。
#    4. 模块化，可整体跳过或只执行指定模块。
#
#  用法:
#    ./shipment_privacy_cleaner.sh                 # 扫描模式(默认，只报告)
#    ./shipment_privacy_cleaner.sh --clean         # 实际清理(逐模块确认)
#    ./shipment_privacy_cleaner.sh --clean --yes   # 全自动清理(不逐个确认)
#    ./shipment_privacy_cleaner.sh --clean --skip=logs,backup   # 跳过模块
#    ./shipment_privacy_cleaner.sh --clean --only=history,ssh    # 只跑模块
#    ./shipment_privacy_cleaner.sh --list          # 列出全部模块
#
#  说明:
#    - 需要 sudo 的模块: logs(系统日志)。
#    - WiFi 与有线网络连接(含已保存密码、静态 IP)一律保留，保证开箱可用。
#    - 主机名 fouruav 默认不改；如需重置请用 --reset-hostname。
#    - 执行完毕后建议: history -c && 注销重新登录，再关机发货。
# =============================================================================

# ----------------------------- 可配置参数(按需修改) -------------------------
# 支持 sudo 运行：优先取被提权前的真实用户主目录
if [ -n "${SUDO_USER:-}" ]; then
  HOME_DIR="$(eval echo "~$SUDO_USER")"
else
  HOME_DIR="${HOME:-/home/fouruav}"
fi
# 需要整体清除的备份/归档(含源码备份，务必清)
ARCHIVE_FILES=(
  "$HOME_DIR/NX_code.zip"
  "$HOME_DIR/odin_ws.zip"
  "$HOME_DIR/actions-runner/actions-runner-linux-arm64-2.336.0.tar.gz"
)

# ----------------------------- 全局变量 -------------------------------------
ACTION="scan"          # scan | clean
ASSUME_YES=0
RESET_HOSTNAME=0
SELECTED_MODULES=""
SKIPPED_MODULES=""
ALL_MODULES="history ssh git browser crypto logs backup"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
MANIFEST="/tmp/shipment_clean_manifest_${TIMESTAMP}.txt"
FOUND_COUNT=0
REMOVED_COUNT=0
RELEASED_BYTES=0

# ----------------------------- 小工具 ---------------------------------------
c_green()  { printf "\033[32m%s\033[0m" "$*"; }
c_yellow() { printf "\033[33m%s\033[0m" "$*"; }
c_red()    { printf "\033[31m%s\033[0m" "$*"; }
c_cyan()   { printf "\033[36m%s\033[0m" "$*"; }
c_bold()   { printf "\033[1m%s\033[0m" "$*"; }

hr() { printf '%*s\n' "${COLUMNS:-80}" '' | tr ' ' '-'; }

# 记录「将删除」清单
manifest_add() {
  echo "$1" >> "$MANIFEST"
}

# 判断某路径是否存在(支持通配符)，输出匹配项数组
# 用法: collect 'path' -> 填充全局数组 FOUND_ITEMS
FOUND_ITEMS=()
collect() {
  FOUND_ITEMS=()
  # shellcheck disable=SC2086
  for p in $1; do [ -e "$p" ] && FOUND_ITEMS+=("$p"); done
}

# 计算人类可读大小
human_size() { du -sh "$1" 2>/dev/null | cut -f1; }

# 单个文件/目录的「扫描 or 清理」核心逻辑
# 用法: handle "模块名" "路径" ["说明"]
handle() {
  local mod="$1" path="$2" desc="${3:-}"
  collect "$path"
  [ "${#FOUND_ITEMS[@]}" -eq 0 ] && return
  local size
  size="$(human_size "${FOUND_ITEMS[0]}")"
  FOUND_COUNT=$((FOUND_COUNT + 1))
  if [ "$ACTION" = "scan" ]; then
    printf "  [%s] %-10s %-7s %s\n" "$(c_green "发现")" "$mod" "$size" "$path ${desc:+($desc)}"
  else
    RELEASED_BYTES=$((RELEASED_BYTES + $(du -sk "${FOUND_ITEMS[0]}" 2>/dev/null | cut -f1)))
    for it in "${FOUND_ITEMS[@]}"; do
      manifest_add "$it"
      rm -rf "$it"
      REMOVED_COUNT=$((REMOVED_COUNT + 1))
    done
    printf "  [%s] %-10s %-7s %s\n" "$(c_red "已删")" "$mod" "$size" "$path"
  fi
}

# 模块执行前确认(仅 clean 模式且非 --yes 时询问；scan 模式一律跳过)
confirm_module() {
  [ "$ACTION" = "scan" ] && return 0
  [ "$ASSUME_YES" -eq 1 ] && return 0
  printf "  %s %s  [y/N] " "$(c_yellow "▶")" "确认清理本模块?"
  read -r ans
  case "$ans" in y|Y|yes|YES) return 0 ;; *) printf "    %s 已跳过本模块\n" "$(c_yellow "⏭")"; return 1 ;; esac
}

# ----------------------------- 各清理模块 ------------------------------------
# 1) shell / 应用历史
mod_history() {
  printf "\n%s %s\n" "$(c_bold "$1")" "$(c_cyan "shell 与应用历史")"
  confirm_module || return 0
  handle history "$HOME_DIR/.bash_history"
  handle history "$HOME_DIR/.python_history"
  handle history "$HOME_DIR/.viminfo"
  handle history "$HOME_DIR/.wget-hsts"
  handle history "$HOME_DIR/.lesshst"
  handle history "$HOME_DIR/.sqlite_history"
  # 注: ~/.Xauthority 按用户要求保留(仅本机 X cookie，无隐私价值)
}

# 2) SSH 私钥 / 公钥 / known_hosts
mod_ssh() {
  printf "\n%s %s\n" "$(c_bold "$1")" "$(c_cyan "SSH 密钥与主机指纹")"
  confirm_module || return 0
  handle ssh "$HOME_DIR/.ssh/id_ed25519" "私钥"
  handle ssh "$HOME_DIR/.ssh/id_ed25519.pub"
  handle ssh "$HOME_DIR/.ssh/id_rsa" "私钥"
  handle ssh "$HOME_DIR/.ssh/id_rsa.pub"
  handle ssh "$HOME_DIR/.ssh/known_hosts"
  handle ssh "$HOME_DIR/.ssh/known_hosts.old"
}

# 3) 全局 git 用户信息
mod_git() {
  printf "\n%s %s\n" "$(c_bold "$1")" "$(c_cyan "git 全局用户信息")"
  confirm_module || return 0
  if [ -f "$HOME_DIR/.gitconfig" ] && grep -q '^\[user\]' "$HOME_DIR/.gitconfig"; then
    if [ "$ACTION" = "scan" ]; then
      FOUND_COUNT=$((FOUND_COUNT + 1))
      printf "  [%s] %-10s %-7s %s\n" "$(c_green "发现")" "git" "" "$HOME_DIR/.gitconfig [user] 段 (name/email)"
    else
      manifest_add "$HOME_DIR/.gitconfig [user] 段"
      git config --global --unset-all user.name 2>/dev/null || true
      git config --global --unset-all user.email 2>/dev/null || true
      git config --global --remove-section user 2>/dev/null || true
      REMOVED_COUNT=$((REMOVED_COUNT + 1))
      printf "  [%s] %-10s %s\n" "$(c_red "已清")" "git" "$HOME_DIR/.gitconfig 中的 user 段"
    fi
  fi
}

# 4) 浏览器 / Copilot 登录态 / QGC 连接记录
mod_browser() {
  printf "\n%s %s\n" "$(c_bold "$1")" "$(c_cyan "浏览器 / Copilot / QGC 登录态")"
  confirm_module || return 0
  handle browser "$HOME_DIR/.mozilla" "Firefox cookies/密码/历史"
  handle browser "$HOME_DIR/.config/google-chrome" "Chrome 数据"
  handle browser "$HOME_DIR/.copilot" "VS Code Copilot GitHub 登录态"
  handle browser "$HOME_DIR/.config/github-copilot"
  handle browser "$HOME_DIR/.config/QGroundControl.org" "QGC 保存的飞行器连接记录(含 IP)"
}

# 5) GPG 密钥 / 证书
mod_crypto() {
  printf "\n%s %s\n" "$(c_bold "$1")" "$(c_cyan "GPG 密钥 / 证书")"
  confirm_module || return 0
  handle crypto "$HOME_DIR/.gnupg" "GPG 密钥环"
  handle crypto "$HOME_DIR/.pki" "用户证书"
}

# 6) 运行日志 / 缓存 / 回收站 / 系统日志
mod_logs() {
  printf "\n%s %s\n" "$(c_bold "$1")" "$(c_cyan "运行日志 / 缓存 / 系统日志")"
  confirm_module || return 0
  # 项目运行日志(保留目录结构，清空内容)
  if [ -d "$HOME_DIR/catkin_ws/src/task_run_scripts/log" ]; then
    if [ "$ACTION" = "scan" ]; then
      FOUND_COUNT=$((FOUND_COUNT + 1))
      printf "  [%s] %-10s %-7s %s\n" "$(c_green "发现")" "logs" "$(human_size "$HOME_DIR/catkin_ws/src/task_run_scripts/log")" "task_run_scripts/log/ 全部运行日志"
    else
      RELEASED_BYTES=$((RELEASED_BYTES + $(du -sk "$HOME_DIR/catkin_ws/src/task_run_scripts/log" 2>/dev/null | cut -f1)))
      find "$HOME_DIR/catkin_ws/src/task_run_scripts/log" -mindepth 1 -maxdepth 1 -exec rm -rf {} +
      manifest_add "task_run_scripts/log/*"
      printf "  [%s] %-10s %s\n" "$(c_red "已清")" "logs" "task_run_scripts/log/ 内容"
    fi
  fi
  handle logs "$HOME_DIR/.ros/log" "ROS 运行日志"
  handle logs "$HOME_DIR/.gazebo/log" "Gazebo 日志"
  handle logs "$HOME_DIR/.cache" "应用缓存(会自动重建)"
  handle logs "$HOME_DIR/.ccache" "编译缓存(会自动重建)"
  handle logs "$HOME_DIR/.local/share/Trash" "回收站"
  # 系统日志(需 sudo)
  if [ "$ACTION" = "clean" ]; then
    if sudo -n true 2>/dev/null; then
      sudo journalctl --rotate >/dev/null 2>&1 || true
      sudo journalctl --vacuum-time=1s >/dev/null 2>&1 || true
      sudo truncate -s 0 /var/log/auth.log /var/log/syslog /var/log/kern.log /var/log/user.log 2>/dev/null || true
      sudo apt-get clean >/dev/null 2>&1 || true
      printf "  [%s] %-10s %s\n" "$(c_red "已清")" "logs" "/var/log journal + auth/syslog (sudo)"
    else
      printf "  [%s] 需要 sudo 才能清系统日志，已跳过。可用: %s\n" "$(c_yellow "提示")" "sudo $0 --clean --only=logs"
    fi
  fi
}

# 7) 网络连接检查(仅显示，不删除 —— WiFi 密码按用户要求保留)
mod_network() {
  printf "\n%s %s\n" "$(c_bold "$1")" "$(c_cyan "网络连接 (仅显示)")"
  local nm_dir="/etc/NetworkManager/system-connections"
  if [ -d "$nm_dir" ]; then
    printf "  %s NetworkManager 保存的连接(全部保留，含 WiFi 密码 / 有线静态 IP):\n" "$(c_yellow "→")"
    ls "$nm_dir" 2>/dev/null | sed 's/^/        - /'
  fi
}

# 7) 备份 / 归档压缩包
mod_backup() {
  printf "\n%s %s\n" "$(c_bold "$1")" "$(c_cyan "备份 / 归档压缩包")"
  confirm_module || return 0
  for f in "${ARCHIVE_FILES[@]}"; do
    handle backup "$f"
  done
}

# 8) 主机名重置(可选)
mod_hostname() {
  printf "\n%s %s\n" "$(c_bold "$1")" "$(c_cyan "主机名重置 (可选)")"
  confirm_module || return 0
  local cur="$(cat /etc/hostname 2>/dev/null)"
  if [ "$RESET_HOSTNAME" -eq 1 ]; then
    local new="${NEW_HOSTNAME:-ubuntu}"
    if [ "$ACTION" = "scan" ]; then
      printf "  [%s] 当前主机名: %s -> 将重置为 %s\n" "$(c_green "发现")" "$cur" "$new"
    else
      if sudo -n true 2>/dev/null; then
        sudo hostnamectl set-hostname "$new"
        echo "$new" | sudo tee /etc/hostname >/dev/null
        sudo sed -i "s/127.0.1.1.*$cur/127.0.1.1 $new/" /etc/hosts 2>/dev/null || true
        printf "  [%s] 主机名已重置: %s -> %s\n" "$(c_red "已改")" "$cur" "$new"
      else
        printf "  [%s] 需要 sudo。可用: %s\n" "$(c_yellow "提示")" "sudo $0 --clean --reset-hostname"
      fi
    fi
  else
    printf "  %s 未启用(--reset-hostname)。当前主机名: %s\n" "$(c_yellow "→")" "$cur"
  fi
}

# ----------------------------- 只报告不删除的疑点 ----------------------------
mod_report_only() {
  printf "\n%s %s\n" "$(c_bold "附:")" "$(c_cyan "需人工确认(脚本不删除)")"
  for f in "$HOME_DIR/timeshare"; do
    [ -e "$f" ] && printf "  - %s  (未知类型 data 文件，请人工确认)\n" "$f"
  done
  # 空目录或乱码目录
  find "$HOME_DIR" -maxdepth 1 -name '*[^[:print:]]*' 2>/dev/null | while IFS= read -r d; do
    printf "  - %q  (非常规文件名，请人工确认)\n" "$d"
  done
  # crontab
  if crontab -l >/dev/null 2>&1; then
    printf "  - 存在 crontab 任务:\n"
    crontab -l | sed 's/^/        /'
  fi
  # bashrc 中可疑的明文 IP/账号
  if [ -f "$HOME_DIR/.bashrc" ]; then
    grep -nE 'export .*[Pp][Aa][Ss][Ss]|export .*[Tt][Oo][Kk][Ee][Nn]|192\.168\.' "$HOME_DIR/.bashrc" 2>/dev/null | sed 's/^/        .bashrc: /'
  fi
}

# ----------------------------- 参数解析 --------------------------------------
usage() {
  sed -n '2,30p' "$0" | sed 's/^# \{0,1\}//'
}

while [ $# -gt 0 ]; do
  case "$1" in
    --list) echo "可用模块: $ALL_MODULES"; exit 0 ;;
    --scan) ACTION="scan" ;;
    --clean) ACTION="clean" ;;
    --yes|-y) ASSUME_YES=1 ;;
    --reset-hostname) RESET_HOSTNAME=1 ;;
    --skip=*) SKIPPED_MODULES="${1#*=}" ;;
    --only=*) SELECTED_MODULES="${1#*=}" ;;
    -h|--help) usage; exit 0 ;;
    *) echo "未知参数: $1"; usage; exit 1 ;;
  esac
  shift
done

[ -n "$SELECTED_MODULES" ] && ALL_MODULES="$SELECTED_MODULES"
for m in $(echo "$SKIPPED_MODULES" | tr ',' ' '); do
  ALL_MODULES="${ALL_MODULES// $m/}"
  ALL_MODULES="${ALL_MODULES//$m /}"
done

# ----------------------------- 主流程 ---------------------------------------
hr
if [ "$ACTION" = "scan" ]; then
  printf "%s\n" "$(c_bold "发货前隐私扫描报告") — 仅扫描，不删除任何内容"
else
  printf "%s\n" "$(c_bold "发货前隐私清理") — 实际删除，审计清单: $MANIFEST"
fi
printf "时间: %s · 用户: %s\n" "$(date '+%Y-%m-%d %H:%M:%S')" "$USER"
hr

MOD_IDX=0
TOTAL_MODS="$(echo "$ALL_MODULES" | wc -w)"
for mod in $ALL_MODULES; do
  MOD_IDX=$((MOD_IDX + 1))
  case "$mod" in
    history)  mod_history  "[$MOD_IDX/$TOTAL_MODS]" ;;
    ssh)      mod_ssh      "[$MOD_IDX/$TOTAL_MODS]" ;;
    git)      mod_git      "[$MOD_IDX/$TOTAL_MODS]" ;;
    browser)  mod_browser  "[$MOD_IDX/$TOTAL_MODS]" ;;
    crypto)   mod_crypto   "[$MOD_IDX/$TOTAL_MODS]" ;;
    logs)     mod_logs     "[$MOD_IDX/$TOTAL_MODS]" ;;
    network)  mod_network  "[$MOD_IDX/$TOTAL_MODS]" ;;
    backup)   mod_backup   "[$MOD_IDX/$TOTAL_MODS]" ;;
    hostname) mod_hostname "[$MOD_IDX/$TOTAL_MODS]" ;;
    *) echo "未知模块: $mod" ;;
  esac
done

mod_report_only

hr
if [ "$ACTION" = "scan" ]; then
  printf "共发现 %s 处隐私痕迹。\n" "$(c_yellow "$FOUND_COUNT")"
  printf "确认无误后执行清理:  %s\n" "$(c_cyan "./shipment_privacy_cleaner.sh --clean --yes")"
  printf "或逐模块确认:        %s\n" "$(c_cyan "./shipment_privacy_cleaner.sh --clean")"
else
  if [ -f "$MANIFEST" ]; then
    cp "$MANIFEST" "$HOME_DIR/shipment_clean_manifest_${TIMESTAMP}.txt"
    printf "已删除/清空 %s 项，释放约 %s。审计清单: %s\n" \
      "$(c_yellow "$REMOVED_COUNT")" "$(c_yellow "$((RELEASED_BYTES / 1024)) MB")" \
      "$HOME_DIR/shipment_clean_manifest_${TIMESTAMP}.txt"
  fi
  printf "请执行: %s，然后注销重新登录，再关机发货。\n" "$(c_yellow "history -c && exec bash")"
fi
hr
