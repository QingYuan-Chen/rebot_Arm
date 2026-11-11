# 筛选结果：在最优抓取候选基础上补齐预抓取位姿。
# 发布在 /grasp/plan 一类话题上，供可视化标记与候选消费者使用。

# 时间戳与坐标系。frame_id 覆盖本消息内所有位姿（candidate、pregrasp_pose、grasp_pose）。
std_msgs/Header header
# 被选中的抓取候选原文，保留类别、置信度、夹爪宽度与来源等信息。
GraspCandidate candidate
# 接近点（预抓取）位姿：位于抓取点沿 TCP 接近轴退开一段偏移（本站默认 0.08 m）处，
# 姿态与 grasp_pose 相同。先到接近点再直线进给，可避免末端侧向撞到目标或桌面。
geometry_msgs/Pose pregrasp_pose
# 抓取点（闭合点）位姿，即 candidate.pose：TCP 到位后闭合夹爪。
geometry_msgs/Pose grasp_pose
# 本次抓取使用的夹爪开口宽度，单位 m；通常等于 candidate.jaw_width。
float32 jaw_width
# 计划是否有效。false 时 grasp_pose/pregrasp_pose 无意义，应读 reason 判断失败原因。
bool valid
# 计划来源标识，字符串枚举，例如 "ordinary_grasp"、"candidate_ik_filter"。
string source
# 无效时的原因说明（英文短句，例如无候选、宽度超行程、深度质量不足）；有效时为空串。
string reason
