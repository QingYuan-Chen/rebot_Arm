# reBotArm Agent 工作区维护手册

`Agent/` 是编码代理共享的项目状态目录，不是 ROS 包、节点或后台服务。
它的目标是让下一位代理或开发者能快速回答四个问题：

1. 当前项目正在做什么？
2. 哪些验收已经完成？
3. 哪些事项被取消、失败或移出范围？
4. 下一步能做什么，哪些硬件动作仍然需要授权？

## 新接手者的阅读顺序

按以下顺序阅读，不要从历史日志开头一路读到底：

1. `CURRENT_STATUS.md`：当前唯一的人类可读状态入口。
2. `EXECUTION_FLOW.md` 的“当前执行队列”：当前任务和边界。
3. `STATE.json`：机器生成的摘要；不要手工修改。
4. `PROJECT_STATUS.md`：P0-P6 历史验收基线，需要核对过去验收时再看。
5. `MEMORY.md`：当前事实、决策和详细历史上下文。
6. `ACTIVITY_LOG.md`、`evidence/`：审计和原始证据，需要追溯具体事件时再看。

## 文件职责和写入规则

| 文件/目录 | 用途 | 谁可以修改 | 规则 |
| --- | --- | --- | --- |
| `CURRENT_STATUS.md` | 当前阶段、当前验收、阻塞、下一步和安全边界 | Agent/开发者 | 只保留当前范围，状态变化时更新 |
| `EXECUTION_FLOW.md` | 当前队列、执行流程和历史队列索引 | Agent/开发者 | 当前队列置顶，旧队列明确标为历史 |
| `STATE.json` | 机器可读摘要 | 仅 `update_state.py` | 禁止手工编辑 |
| `update_state.py` | 记录事件并重新生成 `STATE.json` | 维护脚本 | 任务开始、检查点、完成时运行 |
| `PROJECT_STATUS.md` | P0-P6 历史验收和范围收口记录 | Agent/开发者 | 不把历史失败改写成通过 |
| `MEMORY.md` | 详细事实、决策、阻塞和交接信息 | Agent/开发者 | 顶部维护当前摘要，后部保留历史 |
| `ACTIVITY_LOG.md` | 按时间追加的事件流水 | `update_state.py` | 只追加，不回写历史 |
| `evidence/` | 测试、运行和硬件证据；分为 `current/` 与 `archive/` | Agent/开发者 | 先读 `evidence/README.md`，缺失旧文件时从 Git 历史追溯 |
| `__pycache__/` | Python 自动缓存 | Python | 非项目状态，可清理 |

## 当前与历史的边界

- `CURRENT_STATUS.md` 是当前范围的唯一入口。
- P0-P6 已关闭，但其清单、失败记录和安全边界必须保留作历史基线。
- “范围关闭”不等于“验收通过”；已取消的项目必须保留这个说明。
- 历史证据路径如果在当前工作树不存在，应视为 Git 历史证据，不要当作当前文件。
- 当前 `STATE.json` 中的 P0-P6 完成度是历史加权清单完成度，不代表 RL 收敛或实时硬件验收。

## 标准工作流

开始任务：

```bash
python3 Agent/update_state.py \
  --event start \
  --actor Codex \
  --note "开始处理具体任务"
```

完成可验证检查点：

```bash
python3 Agent/update_state.py \
  --event verified \
  --actor Codex \
  --note "完成软件验证" \
  --verification "填写实际测试命令和结果"
```

结束任务并刷新状态：

```bash
python3 Agent/update_state.py \
  --event complete \
  --actor Codex \
  --note "任务完成" \
  --verification "填写实际验证结果"
```

仅刷新生成状态、不追加事件：

```bash
python3 Agent/update_state.py
```

## 安全和证据规则

- 真机健康 enabled 时，可恢复的任务失败不得直接把机械臂失能在未知姿态；必须保持 enabled hold，或受控回到已记录 baseline 后再 disable。
- 真机默认 disabled；需要 fresh feedback、现场安全检查和用户明确的 `/rebotarm/enable` 授权。
- 软件测试通过不等于真机验收通过。
- 没有代码、测试、运行记录或用户明确来源确认时，不得把验收 checkbox 标为完成。
- 不在 `Agent/` 中保存密码、token、设备私密凭据或用户隐私数据。
