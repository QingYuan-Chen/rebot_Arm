# 可选 STEP 网格生成依赖

仅供 `scripts/import_soft_gripper_meshes.py` 和 `scripts/import_d405_meshes.py`
离线转换 CAD 使用；ROS、RViz、
Dashboard 和 MuJoCo 运行时使用已生成的 STL，不依赖 OpenCascade。
推荐在独立临时环境安装，不修改控制器或视觉环境。

```requirements
cadquery-ocp-novtk==7.8.1.1.post1
numpy==1.26.4
```

通过现有安装工具选择目标 Python：

```bash
python3 scripts/install_python_dependencies.py cad --python /path/to/cad-env/bin/python --show
```

去掉 `--show` 才执行安装。转换命令和坐标约定见
[软指模型](../../implemented/features/soft_gripper_model.md)和
[D405 腕部模型](../../implemented/features/d405_wrist_model.md)。

一次性尺寸检查、装配试验和渲染程序在验证结束后清理，临时 CAD 环境也随之删除。
正式导入脚本、源 CAD 和回归测试保留在仓库；日志、数值报告和效果图留在本机 `log/`，不提交。
