from types import SimpleNamespace as NS

import numpy as np


def test_bad_inference_payload_clears_current_frame():
    from rebotarm_vision.nodes.graspnet_baseline_node import GraspNetBaselineNode
    detected = NS(x_min=0, y_min=0, x_max=2, y_max=2, confidence=.9,
                  class_name='object', has_mask=False, mask_polygon_xy=[])
    cleared, messages = [], []
    values = {'max_input_skew_ms': 100, 'depth_scale_m_per_unit': .001, 'max_grasps': 5,
              'max_jaw_width_m': .08, 'min_depth_m': .05, 'max_depth_m': 1.5}
    node = NS(
        pending_inprocess_detections=NS(header=NS(stamp=1), detections=[detected]),
        _stamp_to_ns=lambda stamp: stamp, last_inprocess_detection_timestamp_ns=0,
        backend=NS(available=True, infer=lambda **_: {'candidates': None}),
        latest_camera_info={'width': 2, 'height': 2, 'fx': 2., 'fy': 2., 'cx': 1., 'cy': 1.},
        latest_camera_info_frame_id='camera', latest_camera_info_timestamp_ns=1,
        color_frame_cache=[(1, np.zeros((2, 2, 3), dtype=np.uint8))],
        depth_frame_cache=[(1, np.ones((2, 2)), 'camera')],
        get_parameter=lambda name: NS(value=values[name]),
        get_logger=lambda: NS(warn=messages.append),
        _publish_empty=lambda **kwargs: cleared.append(kwargs),
    )
    GraspNetBaselineNode._try_process_inprocess_detection(node)
    assert cleared == [{'timestamp_ns': 1, 'frame_id': 'camera'}]
    assert 'inference failed' in messages[0]
