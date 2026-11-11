# 单个 2D 检测结果(YOLO/Gemini2 输出经检测转换器规范化后的形式).
# 所有像素坐标都在彩色图像坐标系下:原点在图像左上角,u 向右,v 向下,
# 尺寸等于彩色图像的宽/高(本站为 640x480).header.frame_id 通常是
# camera_color_frame;需要三维抓取时,上层会按内参和深度图把像素反投影到相机坐标系.

# 时间戳与坐标系.stamp 取产生该帧彩色图的时间,frame_id 为彩色光学坐标系.
std_msgs/Header header
# 类别名（来自模型的类别表，如 "bottle"、"cup"），也是按标签抓取时使用的标签。
string class_name
# 检测置信度，无量纲，取值范围 [0, 1]；低于模型 conf_threshold 的结果不会发布。
float32 confidence
# 检测框中心点像素坐标：center_u 为列（水平方向），center_v 为行（垂直方向），单位 pixel。
int32 center_u
int32 center_v
# 检测框左上角像素坐标，单位 pixel。
int32 x_min
int32 y_min
# 检测框右下角像素坐标（通常为开区间边界），单位 pixel。
int32 x_max
int32 y_max
# 是否带有旋转框（OBB）信息。为 true 时下面 6 个 obb_* 字段有效，否则应忽略。
bool has_obb
# 旋转框中心像素坐标，单位 pixel。
float32 obb_cx
float32 obb_cy
# 旋转框尺寸：宽高，单位 pixel（非轴对齐，随 theta 旋转）。
float32 obb_w
float32 obb_h
# 旋转框角度，单位 rad，取自模型 xywhr 输出的第 5 个分量（绕图像法向的旋转量）。
float32 obb_theta
# 旋转框四个角点，按 (x0,y0,x1,y1,x2,y2,x3,y3) 展开，单位 pixel；无 OBB 时为空数组。
float32[] obb_points_xy
# 是否带有实例分割掩膜。为 true 时 mask_polygon_xy 有效；至少 3 个点（6 个数）才算有效。
bool has_mask
# 掩膜多边形顶点，按 (x0,y0,x1,y1,...) 展开，单位 pixel；无掩膜时为空数组。
float32[] mask_polygon_xy
