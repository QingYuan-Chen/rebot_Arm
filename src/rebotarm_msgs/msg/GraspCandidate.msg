# 单个抓取候选：一个“末端 TCP 应该以什么位姿、张开多大夹爪去抓”的建议。
# 由视觉侧依据检测结果与深度生成（普通抓取、GraspNet 基线、候选 IK 过滤等），
# 下游按置信度/可达性筛选并发布结果。

# 时间戳与坐标系。header.frame_id 是 pose 所在坐标系（通常 camera_depth_frame，
# 经 IK 过滤后可能已变换到机械臂基座系），也是该候选的参考系，必须与 pose 配套解读。
std_msgs/Header header
# 目标类别名，与 Detection2D.class_name 对应，用于按标签匹配目标。
string class_name
# 候选置信度，无量纲，[0, 1]；越大越可信，排序与阈值筛选都基于它。
float32 confidence
# 末端 TCP 目标位姿：位置单位 m，姿态为四元数（无单位，需归一化解读）。
geometry_msgs/Pose pose
# 抓取该目标所需的夹爪开口宽度，单位 m；必须落在夹爪可信行程内（本站约 0.006 ~ 0.085）。
float32 jaw_width
# 目标沿夹爪闭合方向之外的估计长度，单位 m；用于判断夹爪是否会与目标/桌面干涉。
float32 object_length
# 该候选自身是否可用（几何与深度检查通过）。false 的候选仍会出现在数组里，不应被选为有效候选。
bool valid
# 候选来源标识，字符串枚举，例如 "ordinary_grasp_obb_depth"、"ordinary_grasp_mask_depth"、
# "ordinary_grasp_bbox_depth"、"graspnet_baseline"、"candidate_ik_filter"；用于排障溯源。
string source
