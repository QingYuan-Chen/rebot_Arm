# 消息接口包：本包只定义自定义接口（.msg/.srv/.action），不含任何可执行节点。
# 所有上层包（硬件、运动、视觉、示教、遥操作、界面、仿真）都通过这里定义的类型
# 交换数据，因此改动本文件等于改动对外契约，必须与各使用方同步。
cmake_minimum_required(VERSION 3.8)
project(rebotarm_msgs)

# ament_cmake：ROS 2 的 CMake 构建基础设施（install/export/ament_package 等）。
find_package(ament_cmake REQUIRED)
# rosidl_default_generators：提供 rosidl_generate_interfaces()，负责把接口文件
# 生成 C++/Python 绑定（Python 侧 import 路径为 rebotarm_msgs.msg/srv/action）。
find_package(rosidl_default_generators REQUIRED)
# 以下四个是接口字段里直接引用的类型来源，必须同时出现在 DEPENDENCIES 与 package.xml：
# builtin_interfaces 提供 Time（时间戳，秒+纳秒）；geometry_msgs 提供 Pose/PoseStamped；
# std_msgs 提供 Header；sensor_msgs 提供纯规划虚拟起点 JointState；
# trajectory_msgs 提供 JointTrajectory。
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# 注册本包全部接口，按 msg/（数据消息）、srv/（请求-响应服务）、action/（长时动作）
# 三类依次列出；注意：测试会逐行扫描本调用中形如 "msg/xxx.msg" 的整行引号字符串来
# 核对文件是否存在，因此每个接口必须独占一行，且行内不得插入其它内容（含行尾注释）。
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Detection2D.msg"
  "msg/Detection2DArray.msg"
  "msg/GraspCandidate.msg"
  "msg/GraspCandidateArray.msg"
  "msg/GraspPlan.msg"
  "msg/ExecutionState.msg"
  "msg/TaskStatus.msg"
  "msg/JointMotorCmd.msg"
  "msg/JointMotorState.msg"
  "msg/ArmStatus.msg"
  "srv/PlanGraspForLabel.srv"
  "srv/SetMode.srv"
  "srv/CalibrationCommand.srv"
  "srv/SetZero.srv"
  "srv/SetTeachRecordPath.srv"
  "srv/MoveToPoseIK.srv"
  "srv/ExecutePose.srv"
  "srv/PublishTrajectoryPreview.srv"
  "srv/SetGripper.srv"
  "srv/GraspGripper.srv"
  "action/ExecuteGrasp.action"
  "action/MoveRelative.action"
  "action/MoveToPose.action"
# DEPENDENCIES 声明生成代码要链接/导入的上游接口包，必须与上面 find_package 的
# 列表完全一致，漏写会导致生成的消息模块在运行时 import 失败。
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs sensor_msgs trajectory_msgs
)

# 把 rosidl 运行时依赖导出给下游包，下游只需 <depend>rebotarm_msgs</depend> 即可。
ament_export_dependencies(rosidl_default_runtime)
ament_package()
