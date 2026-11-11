# 同一帧/同一次规划产生的全部抓取候选，按得分从高到低排序。
# 发布在 /grasp/candidates 一类话题上，供候选筛选和显示工具消费。

# 时间戳与坐标系。frame_id 是所有候选 pose 的公共参考系。
std_msgs/Header header
# 候选列表，已按得分降序排列；同一个候选同时包含 header，便于单独转发。
GraspCandidate[] candidates
# 最优候选在 candidates 中的下标：列表非空时为 0（因为已排序），列表为空时为 -1。
# 使用前必须先判 < 0，直接索引空列表会越界。
int32 best_index
