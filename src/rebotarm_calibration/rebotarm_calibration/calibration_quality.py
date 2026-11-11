"""标定样本覆盖度、训练集重采样与残差异常诊断；阈值和样本保留策略由工作流执行。"""
import numpy as np
import cv2
from .handeye_residual import transform_matrix, rotation_observability
from .handeye_solver import solve_eye_in_hand, rotation_angle_deg


def sample_coverage(samples, *, mode='handeye'):
    values = [transform_matrix(sample['base_to_end']) for sample in samples]
    translation = max((float(np.linalg.norm(a[:3, 3] - b[:3, 3]))
                       for i, a in enumerate(values) for b in values[i+1:]), default=0.)
    rotation = max((rotation_angle_deg(a[:3, :3].T @ b[:3, :3])
                    for i, a in enumerate(values) for b in values[i+1:]), default=0.)
    observable = rotation_observability(values)
    gates = {'sample_count': len(samples) >= 5, 'translation_span': translation >= .05,
             'rotation_span': rotation >= (30. if mode == 'tcp' else 20.), 'observability': observable['pass']}
    if mode == 'tcp':
        gates.pop('translation_span')
    return {'mode': mode, 'sample_count': len(samples), 'translation_span_m': translation,
            'rotation_span_deg': rotation, 'observability': observable, 'gates': gates,
            'ready': all(gates.values())}


def bootstrap_diagnostic(samples, method, reference, *, iterations=64, seed=0):
    """只对训练集重采样，报告求解离散度及退化抽样次数。

    固定 seed 便于复算；统计描述采样波动，系统测量偏差需结合独立标定证据评估。
    """
    rng = np.random.default_rng(seed)
    translations, rotations = [], []
    for _ in range(iterations):
        subset = [samples[index] for index in rng.integers(0, len(samples), len(samples))]
        try:
            solved = solve_eye_in_hand(subset, method=method)
            translations.append(solved[:3, 3])
            rotations.append(rotation_angle_deg(reference[:3, :3].T @ solved[:3, :3]))
        except (ValueError, cv2.error, np.linalg.LinAlgError):
            continue
    report = {'method': 'training_bootstrap', 'seed': seed, 'iterations': iterations,
              'successful_draws': len(translations), 'failed_draws': iterations-len(translations),
              'available': len(translations) >= max(10, iterations // 2),
              'interpretation': 'empirical resampling spread; excludes systematic measurement bias'}
    if report['available']:
        report.update(translation_std_m=np.std(translations, axis=0, ddof=1).tolist(),
                      translation_percentile_95_m=np.percentile(translations, [2.5, 97.5], axis=0).tolist(),
                      rotation_deviation_p95_deg=float(np.percentile(rotations, 95)))
    return report


def residual_diagnostics(samples):
    """标记可疑残差供人工查看，保留原样本和质量门。

    相对阈值采用中位数加三倍尺度化 MAD，并保留分辨率下限；绝对门限用于识别
    整组共同偏移，避免 MAD 为零时遗漏系统性残差。
    """
    metrics = {'position_residual_m': (.010, .0001),
               'rotation_residual_deg': (3., .01)}
    cutoffs = {}
    for key, (limit, floor) in metrics.items():
        values = np.asarray([sample[key] for sample in samples], dtype=float)
        if not len(values) or not np.all(np.isfinite(values)) or np.any(values < 0):
            raise ValueError('residual diagnostics require finite non-negative samples')
        median = float(np.median(values))
        mad = float(np.median(np.abs(values - median)))
        cutoffs[key] = {'median': median, 'mad': mad, 'absolute_limit': limit,
                        'robust_cutoff': median + max(3 * 1.4826 * mad, floor)}
    flagged = []
    for sample in samples:
        reasons = []
        for key, threshold in cutoffs.items():
            if sample[key] > threshold['absolute_limit']:
                reasons.append(key + ':absolute_limit')
            if sample[key] > threshold['robust_cutoff']:
                reasons.append(key + ':relative_outlier')
        if reasons:
            flagged.append({'label': sample['label'], 'reasons': reasons,
                            **{key: sample[key] for key in metrics}})
    return {'policy': 'diagnostic_only_no_samples_removed', 'cutoffs': cutoffs,
            'flagged_samples': flagged}
