# 任务级状态通告：以"任务"为单位对外汇报进度，粒度比 ExecutionState 更粗，
# 便于界面显示一条完整作业（例如"抓取瓶子"）的当前阶段。

# 时间戳取状态更新时刻；frame_id 不使用。
std_msgs/Header header
# 任务标识，由发起方生成（同一任务的多次状态更新保持同一个 id），用于区分并发/连续任务。
string task_id
# 任务类型标识字符串（例如抓取、放置、移动一类的分类），由上层流程定义。
string task_type
# 本任务的目标标签，与 Detection2D.class_name / GraspCandidate.class_name 对应。
string target_label
# 当前阶段名（字符串枚举），描述任务推进到哪一步（例如感知、规划、接近、闭合、回撤）。
string phase
# 任务是否仍在进行：true 表示占用执行通道，新任务应等待或拒绝。
bool busy
# 任务是否成功结束；仅在终态有意义。
bool success
# 附加说明文本（失败原因或阶段描述），无内容时为空串。
string message
