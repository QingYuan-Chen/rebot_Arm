# 视觉模型资源

本目录保存视觉包安装时使用的模型资源，与 Python 推理实现分开管理。

- `yolo26s-seg.pt`：仓库已有的 YOLO 分割权重，迁移时保持文件内容不变。
- `yolo26m-seg-fp16-b1-640-linux.engine`：可选的本机 TensorRT 引擎，不随本次迁移生成。

`setup.py` 将存在的上述文件安装到 `share/rebotarm_vision/models/`。
launch 通过 ament 包索引使用安装路径；构建时允许模型缺失，运行时必须提供有效模型。
其他模型可通过 `yolo_model_path` 指定外部路径，无需复制到包内。

GraspNet 第三方源码与检查点仍由 `GRASPNET_MODEL_ROOT` 和
`GRASPNET_CHECKPOINT_PATH` 指定；本目录不复制第三方训练仓库或检查点。
