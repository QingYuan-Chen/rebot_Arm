# 执行状态通告：把"当前在做什么、结果如何"以字符串状态对外广播，
# 供界面与上层流程编排观测一次抓取/执行的阶段推进。

# 时间戳取状态变化或周期发布的时刻；frame_id 不使用。
std_msgs/Header header
# 当前执行状态（字符串枚举，小写），例如 "idle"、"executing"、"preview_ready"；
# 与同包的动作 feedback.state 使用同一套命名。
string state
# 状态补充说明：可为失败原因、当前步骤名等自由文本；无补充时为空串。
string detail
# 本次执行是否成功。仅在终态有意义；进行中为 false，不要用它判断"未开始"。
bool success
