"""低层电机指令透传：把外部直接下发的原始电机命令转交给硬件管理层。

职责与位置
    本模块属于硬件包，是"绕过规划与轨迹"的底层直控入口：为每个关节与夹爪订阅原始
    电机命令话题，判断此刻是否允许透传，然后交给 HardwareManager 下发。它不做插值、
    不做限位裁剪、不做碰撞或奇异检查——那些属于上层运动包的职责，这里只把守"是否
    允许此刻透传"这一道门。

对外接口（话题名统一为 /<命名空间>/<名称>）
    /<namespace>/joints/<joint>/cmd  每个关节一条低层命令（关节名取自硬件的 joint_names）；
    /<namespace>/gripper/cmd         夹爪低层命令，仅当硬件已初始化夹爪时才订阅。

仲裁语义
    回调只提交有有效期的请求。运行时对六轴、夹爪分别原子仲裁：reject 拒绝
    占用资源的透传；preempt 确认六轴保持或夹爪中性释放后接替旧任务。
    不在 ROS 线程先查状态再发送，避免检查与写入之间发生目标接替。

安全约束
    透传本身不负责使能：硬件未使能、反馈不新鲜或模式不支持时，底层下发会抛异常，
    这里只记录警告并照常刷新状态，不让异常扩散到执行器线程；每条命令（含失败）结束后
    都会刷新一次 arm_status，避免锁存的状态发布滞后于真实硬件。
"""

from __future__ import annotations

from rclpy.qos import QoSProfile, ReliabilityPolicy
from rebotarm_msgs.msg import JointMotorCmd


class MotorPassthrough:
    """按硬件实际的关节列表建立低层命令订阅，并持有这些订阅的生命周期。

    订阅在构造时一次性建好，与节点同生命周期，不提供取消逻辑；回调都注册在节点的
    可重入回调组里，因此不同关节的命令可能并发进入，串行化由硬件管理器内部的锁与
    统一控制循环负责。
    """

    def __init__(self, node, hardware, namespace: str, arbitration: str) -> None:
        """建立关节与夹爪的低层命令订阅。

        参数：
            node        宿主节点，需提供 logger、reentrant_group 回调组与
                        publish_arm_status()；
            hardware    HardwareManager 实例，真正执行命令下发；
            namespace   话题名前缀（不含斜杠），例如 "rebotarm"；
            arbitration 轨迹运行期间的处置策略："reject" 拒绝，"preempt" 抢占。
        """
        self._node = node
        self._hardware = hardware
        self._arbitration = arbitration
        # 队列深度 10 加可靠传输：低层指令是有状态的，丢帧比延迟更危险
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._subscriptions = []
        for joint_name in hardware.joint_names:
            # 每个关节一条独立话题，回调里通过闭包绑定对应关节名
            self._subscriptions.append(
                node.create_subscription(
                    JointMotorCmd,
                    f"/{namespace}/joints/{joint_name}/cmd",
                    self._make_joint_callback(joint_name),
                    qos,
                    callback_group=node.reentrant_group,
                )
            )
        if hardware.has_gripper:
            self._subscriptions.append(
                node.create_subscription(
                    JointMotorCmd,
                    f"/{namespace}/gripper/cmd",
                    self._gripper_callback,
                    qos,
                    callback_group=node.reentrant_group,
                )
            )

    def _make_joint_callback(self, joint_name: str):
        """生成绑定到指定关节的命令回调（闭包，省去回调内再查名字）。"""

        def _callback(msg: JointMotorCmd) -> None:  # 为每个关节生成一个独立的回调函数
            try:
                self._hardware.send_joint_motor_cmd(joint_name, msg, arbitration=self._arbitration)
            except Exception as exc:
                # 底层拒绝（未使能、模式不支持等）只记警告，不影响其它关节的命令
                self._node.get_logger().warn(f"joint cmd failed for {joint_name}: {exc}")
            finally:
                # 无论成败都刷新状态发布，保证 arm_status 不滞后于硬件
                self._node.publish_arm_status()

        return _callback

    def _gripper_callback(self, msg: JointMotorCmd) -> None:
        """夹爪低层命令回调：提交独立资源的仲裁请求。"""
        try:
            self._hardware.send_gripper_motor_cmd(msg, arbitration=self._arbitration)
        except Exception as exc:
            self._node.get_logger().warn(f"gripper cmd failed: {exc}")
        finally:
            self._node.publish_arm_status()
