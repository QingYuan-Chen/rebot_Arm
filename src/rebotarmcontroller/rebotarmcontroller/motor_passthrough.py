"""低层电机指令透传：把外部直接下发的原始电机命令转交给硬件管理层。

职责与位置
    本模块属于硬件包，是"绕过规划与轨迹"的底层直控入口：为每个关节与夹爪订阅原始
    电机命令话题，判断此刻是否允许透传，然后交给 HardwareManager 下发。它不做插值、
    不做限位裁剪、不做碰撞或奇异检查——那些属于上层运动包的职责，这里只把守"是否
    允许此刻透传"这一道门。

对外接口（话题名统一为 /<命名空间>/<名称>）
    /<namespace>/joints/<joint>/cmd  每个关节一条低层命令（关节名取自硬件的 joint_names）；
    /<namespace>/gripper/cmd         夹爪低层命令，仅当硬件已初始化夹爪时才订阅。

仲裁语义（取值来自节点参数 cmd_arbitration，构造时传入）
    硬件状态机处于 TRAJ_RUNNING（正在执行轨迹）时：
        "reject"（默认）    丢弃本次命令并打印警告，保护正在执行的轨迹不被打断；
        其它取值（"preempt"）先抢占：置位底层位置控制器的停止事件、清掉 moving 标志，
        让轨迹发送循环停下后再下发本次命令，避免两个控制源同时向同一批电机写指令。
    夹爪通道不做这层仲裁，命令直接下发。

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
            # 轨迹运行期间低层指令会与轨迹控制回路抢同一条电机通道，必须先仲裁
            if self._hardware.state_machine == "TRAJ_RUNNING":
                if self._arbitration == "reject":
                    # 默认策略：直接丢弃，保证正在执行的轨迹不被打断
                    self._node.get_logger().warn(
                        f"rejecting /joints/{joint_name}/cmd while trajectory is running"
                    )
                    return
                self._node.get_logger().warn(
                    f"preempting trajectory for /joints/{joint_name}/cmd"
                )
                # 抢占：置位底层位置控制器的停止事件并清掉 moving 标志，让轨迹发送循环退出
                self._hardware.endpos_ctrl._stop_send.set()
                self._hardware.endpos_ctrl._moving = False

            try:
                self._hardware.send_joint_motor_cmd(joint_name, msg)
            except Exception as exc:
                # 底层拒绝（未使能、模式不支持等）只记警告，不影响其它关节的命令
                self._node.get_logger().warn(f"joint cmd failed for {joint_name}: {exc}")
            finally:
                # 无论成败都刷新状态发布，保证 arm_status 不滞后于硬件
                self._node.publish_arm_status()

        return _callback

    def _gripper_callback(self, msg: JointMotorCmd) -> None:
        """夹爪低层命令回调：不参与轨迹仲裁，直接下发。"""
        try:
            self._hardware.send_gripper_motor_cmd(msg)
        except Exception as exc:
            self._node.get_logger().warn(f"gripper cmd failed: {exc}")
        finally:
            self._node.publish_arm_status()
