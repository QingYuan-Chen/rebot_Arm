"""控制器生命周期与关闭时的安全协调。

``ControllerLifecycle`` 保存对外生命周期及执行状态，调用方持有生命周期锁时
更新；``set_execution`` 将轨迹与保持状态同步到生命周期。``shutdown_policy``
协调节点关闭：健康使能状态必须先保持并等待操作员显式失能，回位失败也不
自动失能。保护性失能由反馈或设备故障路径负责，本模块不直接访问设备。
"""

LIFECYCLE_STATES = frozenset({
    'DISCONNECTED', 'CONNECTED_DISABLED', 'ENABLING', 'ENABLED_HOLD',
    'TRAJECTORY_RUNNING', 'DISABLING',
})
EXECUTION_STATES = frozenset({'IDLE', 'TRAJ_RUNNING', 'LOWLEVEL_STREAMING', 'GRAVITY_COMP'})


class ControllerLifecycle:
    """保存一个设备会话的生命周期和公开执行状态。"""

    def __init__(self):
        self.state = 'DISCONNECTED'
        self.execution = 'IDLE'

    def set_state(self, state):
        if state not in LIFECYCLE_STATES:
            raise ValueError(f'unsupported lifecycle state: {state}')
        self.state = state

    def set_execution(self, state, *, enabled):
        if state not in EXECUTION_STATES:
            raise ValueError(f'unsupported state machine value: {state}')
        self.execution = state
        if enabled and self.state != 'DISABLING':
            if state == 'TRAJ_RUNNING':
                self.state = 'TRAJECTORY_RUNNING'
            elif state == 'IDLE':
                self.state = 'ENABLED_HOLD'


def shutdown_policy(hardware, *, safe_home, logger):
    """仅在已确认失能时释放硬件；使能时保留控制以等待人工恢复。

    可选停靠成功也不等同于验证本轮安全基线；操作员需确认现场并显式失能。
    失败后尝试停止并保持，若设备或反馈故障则由硬件保护路径处置。
    返回 False 时调用方必须保留节点、执行器和设备连接。
    """
    if hardware is None:
        return True
    if hardware.connected and hardware.enabled:
        if safe_home is not None:
            try:
                safe_home()
            except Exception as exc:
                logger.error(f'shutdown safe_home failed: {exc}')
        try:
            hardware.stop_gravity_compensation()
            hardware.ensure_pos_vel_control()
            hardware.stop_active_motion()
        except Exception as exc:
            logger.error(f'shutdown hold failed; keeping controller active: {exc}')
        if hardware.enabled:
            logger.warn('shutdown deferred: arm remains enabled; confirm safety and explicitly disable')
            return False
    hardware.shutdown()
    return True
