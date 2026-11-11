"""示教录制与回放工具的兼容导出入口.

数据契约,文件读写,准备,质量分析和门控由各职责模块实现;此处保留现有导入路径,
供调用方逐步迁往对应所有者,模块本身只汇总符号.
"""

from __future__ import annotations

from .teach_models import (
    PreparedTeachReplay,
    ReplayStartBand,
    ReplayStartDecision,
    RetimedTeachPoint,
    TeachDryRunDecision,
    TeachRecordInfo,
    TeachSample,
    TeachTrajectoryEvent,
    TeachTrajectoryQuality,
)
from .teach_preparation import (
    build_replay_start_soft_points,
    interpolate_joint_positions,
    lowpass_filter_teach_samples,
    prepare_teach_replay_samples,
    prepared_teach_replay_to_dict,
    resample_teach_samples,
    retime_teach_samples,
    smooth_teach_samples,
    teach_trajectory_preview_to_dict,
)
from .teach_quality import (
    analyze_teach_trajectory,
    detect_teach_record_anomalies,
    inspect_teach_record,
    list_teach_record_files,
    teach_record_info_to_dict,
    teach_trajectory_quality_to_dict,
)
from .teach_record_io import (
    decode_teach_sample,
    encode_teach_sample,
    is_quit_key,
    load_teach_samples,
    prepared_record_path,
    write_prepared_teach_record,
)
from .teach_replay_gates import (
    classify_replay_start,
    compute_auto_align_duration,
    estimate_teach_replay,
    normalize_teach_replay_settings,
    validate_teach_dry_run_request,
    validate_teach_replay_execute_request,
    validate_teach_replay_stop_request,
)
