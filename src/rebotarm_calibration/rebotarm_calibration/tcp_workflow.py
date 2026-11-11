"""固定接触点的 TCP 枢轴标定：训练姿态求偏移和接触点，独立姿态检查残差。"""
import numpy as np
from .tcp_calibration import analyze_tcp_pivot_samples
from .handeye_residual import transform_matrix


def solve_tcp_dataset(dataset):
    groups = [dataset[key] for key in ('training_samples', 'validation_samples')]
    if any(len(group) < 5 for group in groups):
        raise ValueError('TCP requires five training and five validation samples')
    seen = set()
    converted = []
    matrices = []
    for group in groups:
        values=[]
        poses=[]
        for sample in group:
            pose=transform_matrix(sample['base_to_end'])
            key=tuple(np.round(pose.ravel(),8))
            if key in seen:
                raise ValueError('duplicate TCP robot pose')
            seen.add(key)
            transform=sample['base_to_end']
            values.append({'end_link_position':transform['translation'],
                           'end_link_orientation_xyzw':transform['rotation_xyzw']})
            poses.append(pose)
        converted.append(values);matrices.append(poses)
    train=analyze_tcp_pivot_samples(converted[0])
    validation_diversity=analyze_tcp_pivot_samples(converted[1])
    offset=np.asarray(train['tcp_offset_xyz']);pivot=np.asarray(train['pivot_position_base_xyz'])
    # 验证沿用训练求得的同一个接触点（m）；重新拟合验证点会掩盖两组之间的偏移。
    errors=np.asarray([np.linalg.norm(pose[:3,3]+pose[:3,:3]@offset-pivot) for pose in matrices[1]])
    rms=float(np.sqrt(np.mean(errors**2)));maximum=float(errors.max())
    gates={'training':train['passed'],
           'validation_observability':all(validation_diversity['gates'][key] for key in ('full_rank','condition_number','rotation_span')),
           'validation_rms':rms <= .003, 'validation_maximum':maximum <= .006}
    from .provenance import dataset_fingerprint, runtime_provenance
    return {'dataset_sha256': dataset_fingerprint(dataset), 'solver_provenance': runtime_provenance({}),
            'schema_version':1,'kind':'tcp_pivot','passed':all(gates.values()),'accepted':False,
            'deployed':False,'gates':gates,'tcp_offset_xyz':offset.tolist(), 'training':train,
            'validation':{'reference_source':'training_pivot','rms_residual_m':rms,
                          'maximum_residual_m':maximum,'per_sample_residual_m':errors.tolist()},
            'limits':{'validation_rms_m':.003,'validation_maximum_m':.006}}
