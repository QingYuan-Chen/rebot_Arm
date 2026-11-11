"""保留轨迹 Action 在终态迁移失败时的错误结果。

职责：补齐 Jazzy ActionServer 的异常结果收尾；常规执行仍由原服务端处理。
接口：TrajectoryActionServer，用于带 error_code/error_string 的轨迹 Action。
约束：终态迁移持续失败时，以 UNKNOWN 和明确失败原因回复并拒绝后续目标；
    原生句柄及结果保留到服务端关闭，重新创建服务端后恢复接纳。
    轨迹停止与资源令牌仍归执行组件；正常结果沿用原 result_timeout。
结构：目标故障准入 → 原执行回调包装 → 故障结果结算。
"""

from action_msgs.msg import GoalStatus
from rclpy.action import ActionServer, GoalResponse
from rclpy.executors import await_or_execute


class TrajectoryActionServer(ActionServer):
    """终态错误可见返回，避免客户端的 get_result 一直等待。"""

    def __init__(self, *args, **kwargs):
        self._terminal_fault = None
        super().__init__(*args, **kwargs)
        goal_callback = self._goal_callback

        async def accept(request):
            if self._terminal_fault is not None:
                self._logger.error(f'trajectory Action requires restart: {self._terminal_fault}')
                return GoalResponse.REJECT
            return await await_or_execute(goal_callback, request)

        self.register_goal_callback(accept)

    async def _execute_goal(self, execute_callback, goal_handle):
        result = None
        callback_error = None

        async def execute(handle):
            nonlocal result, callback_error
            try:
                result = await await_or_execute(execute_callback, handle)
            except Exception as error:
                callback_error = error
                raise
            return result

        try:
            await super()._execute_goal(execute, goal_handle)
        except Exception as error:
            # Jazzy 在回调返回后再次 abort；该步骤抛错会跳过原实现的结果 Future 结算。
            if result is None:
                result = self._action_type.Result()
                if callback_error is not None:
                    result.error_string = f'{type(callback_error).__name__}: {callback_error}'
            result.error_code = -1  # FJT / ExecuteTeleopMotion 的 INVALID_GOAL。
            result.error_string = (f'{result.error_string}; Action terminal transition failed: '
                f'{type(error).__name__}: {error}; previous status={goal_handle.status}; '
                'trajectory Action requires restart')
            self._terminal_fault = result.error_string
            self._logger.error(result.error_string)
            key = bytes(goal_handle.goal_id.uuid)
            # 活动句柄仍被 rcl_action 跟踪，提前释放会留下悬挂引用；随服务端统一释放。
            # UNKNOWN 明确表示协议终态未确认，失败详情供客户端立即处理及迟到查询。
            response = self._action_type.Impl.GetResultService.Response()
            response.status, response.result = GoalStatus.STATUS_UNKNOWN, result
            self._result_futures[key].set_result(response)
