# 一帧图像上的全部 2D 检测结果(一条话题消息对应一帧,避免逐检测发布导致时间戳无法对齐).

# 时间戳与坐标系,语义与 Detection2D 一致(彩色图像坐标系,原点左上角).
std_msgs/Header header
# 该帧的检测列表；为空表示本帧没有检出任何目标。
Detection2D[] detections
