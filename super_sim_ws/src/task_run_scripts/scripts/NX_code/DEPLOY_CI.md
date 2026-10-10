# 🔐 CI 自动发布 + 双 Clone 部署（方案 C）

本方案实现「**开发机拉 master（源码）、部署机拉 deploy（编译产物）**」两个 clone 的分离工作流，
配合 GitHub Actions 自动编译发布，最省心、不泄露源码。

---

## 1. 工作流总览

```
┌──────────┐  push master   ┌──────────────────────┐  编译产物   ┌──────────────┐
│ 开发机 NX │ ─────────────▶ │ GitHub Actions        │ ─────────▶ │ deploy 分支   │
│ (源码修改) │                │ self-hosted runner    │            │ (仅 .so/资源) │
└──────────┘                │ 自动跑 build_secure.sh │            └──────────────┘
                            └──────────────────────┘                   │
                                                              ┌────────▼─────────┐
                                                              │ 部署机 (其他 NX)   │
                                                              │ git clone -b deploy│
                                                              └──────────────────┘
```

- **开发机**：克隆 `master` 分支 = 完整 `.py` 源码，可改代码、重编、push。
- **部署机**：克隆 `deploy` 分支 = 只有 Cython 编译的 `.so` 与配置文件，**不含源码**，开箱即用。

---

## 2. 一次性配置（只需一次）

### 2.1 注册 self-hosted runner（必须，在 NX 开发机上）

> 编译产物是 `cpython-38-aarch64-linux-gnu.so`，只能在 **aarch64 + Python 3.8** 环境生成，
> 因此用 NX 机本身作为 GitHub Actions runner。

1. 打开 `github.com/jihaofly/NX_code` → **Settings → Actions → Runners** → **New self-hosted runner**
2. 选择 **Linux → ARM64**，复制页面给出的注册命令，在 NX 终端执行（会下载 runner、生成 token）
3. 注册成功后，把该 runner 打上标签：`self-hosted`、`linux`、`aarch64`
4. 确认 runner 处于 **Idle** 状态

### 2.2 确认依赖已装（NX 上通常已具备）

```bash
python3 -c "import Cython" || pip3 install cython
command -v gcc || sudo apt-get install -y gcc
```

---

## 3. 日常使用

### 开发机（master，源码）

```bash
# 克隆 / 已有 clone 保持 master
git clone git@github.com:jihaofly/NX_code.git nx_dev
cd nx_dev && git checkout master

# 改完代码后正常推送，CI 会自动发布
git add -A && git commit -m "xxx" && git push origin master
```

### 部署机（deploy，编译产物，无源码）

```bash
# 全新部署：直接克隆 deploy 分支
git clone -b deploy git@github.com:jihaofly/NX_code.git nx_deploy

# 更新：拉取最新部署包
cd nx_deploy && git pull origin deploy
```

deploy 分支内即为 `build_secure.sh` 的输出，启动方式与源码版完全一致：

```bash
bash start_qgc_servers.sh        # QGC 三大服务（推流/触控/指令）
bash start_fastlivo2.sh false    # Mid360 雷达建图
```

> 部署机的 `.desktop` / `.sh` 已自动改为调用 `*_launcher.py`（import `.so`），无需源码。

---

## 4. 手动重新发布

不 push 代码，也可以手动触发一次编译发布：
GitHub 仓库 → **Actions** → **🔐 构建并发布部署包** → **Run workflow**。

---

## 5. 关键注意点

| 事项 | 说明 |
|------|------|
| 架构绑定 | `.so` 为 aarch64 + Py3.8，仅能在同平台 NX 上运行 |
| 源码保护 | deploy 分支只含编译产物，`.py` 源码不会发布 |
| `.gitignore` | 发布时已移除忽略 `*.so` 的规则，确保产物入库 |
| runner 状态 | 若 runner 离线，push 后 workflow 会一直 pending，需先修复 runner |
| 回滚 | 每次发布覆盖 `deploy` 分支（force push）；本地可保留旧 commit 备份 |
