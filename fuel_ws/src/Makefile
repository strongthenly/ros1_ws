# fuel_bridge / FUEL 接入 —— 开发入口
# 用法: make test    （跑冒烟测试，需在 WSL ros1-twin 内执行）
#       make lint    （Python 语法检查）

.PHONY: test lint

# 冒烟测试: 输入桥转发 / seed 悬停 / 未对齐防撞 / 坐标系朝向对齐
test:
	bash fuel_bridge/test/smoke_test.sh

# 语法检查（用 ast.parse，不留 __pycache__）
lint:
	python3 -c "import ast,glob;[ast.parse(open(f).read()) for f in glob.glob('fuel_bridge/scripts/*.py')];print('scripts py syntax ok')"
	python3 -c "import xml.etree.ElementTree as E; E.parse('fuel_bridge/launch/fuel_real.launch'); print('launch XML ok')"
