# Star Arm 102-LD → reBot B601 实时主从跟随

本目录是独立 Python SDK 工具，不接入 `robot_Arm` 的 ROS 2 启动链，也不会由主项目自动启动。

- 来源仓库：`servodevelop/Star-Arm-102`
- 来源分支：`codex/python-sdk-mapping-audit`
- 来源提交：`5ea7e51`（`fix: 强化主从跟随安全控制`）
- 工具入口：`Python_SDK/rebot_b601_mapping/`
- 完整说明：[`Python_SDK/rebot_b601_mapping/README.md`](Python_SDK/rebot_b601_mapping/README.md)

运行前必须先检查 `/dev/ttyUSB0` 和 `/dev/ttyACM0` 的占用。没有明确实机运动授权时，
不得使用 `--confirm-live-motion`。

安装与离线测试：

```bash
cd /home/a/project/rebot_Arm/star_arm_102_rebot_b601_follow
python3 -m venv .venv
.venv/bin/python -m pip install -r \
  Python_SDK/rebot_b601_mapping/requirements-dev.txt
PYTHONPATH=Python_SDK .venv/bin/python -m pytest \
  Python_SDK/rebot_b601_mapping/tests -q
```
