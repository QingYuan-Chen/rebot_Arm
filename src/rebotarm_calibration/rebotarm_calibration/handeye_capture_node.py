"""只读手眼采集服务：独立接收图像/内参与 TF，业务线程执行检测和会话落盘。

接收快照保留源时间，采集仍要求请求后的新帧及连续稳定窗口；不调用电机服务或动作。
"""
import json
import base64
import cv2
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.validate_full_topic_name import validate_full_topic_name
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from cv_bridge import CvBridge
from tf2_ros import TransformException
from .capture_input_receiver import CaptureInputReceiver
from rebotarm_msgs.srv import CalibrationCommand

from .aruco_pose import detect_aruco_pose
from .handeye_residual import transform_matrix
from .handeye_solver import rotation_angle_deg
from .stability_window import StabilityWindow
from .session_store import SessionStore
from .provenance import runtime_provenance


def stamp_ns(stamp):
    return int(stamp.sec) * 10**9 + int(stamp.nanosec)


class WaitingObservation(Exception):
    """No new observation yet; bounded by capture deadline and gap gate."""


class HandeyeCaptureNode(Node):
    def __init__(self):
        super().__init__('rebotarm_handeye_capture')
        defaults = {'image_topic': '/camera/color/image_raw',
                    'camera_info_topic': '/camera/color/camera_info',
                    'session_directory': '~/.ros/rebotarm_calibration',
                    'maximum_age_sec': .5, 'maximum_info_skew_sec': .1,
                    'capture_timeout_sec': 5., 'stability_window_sec': .4, 'maximum_sample_gap_sec': .2,
                    'maximum_translation_m': .001, 'maximum_rotation_deg': .5,
                    'maximum_reprojection_px': 1., 'minimum_area_px2': 400.,
                    'maximum_distance_m': 2.}
        self.settings = {}
        for key, default in defaults.items():
            value = self.declare_parameter(key, default).value
            if isinstance(default, float) and (not math.isfinite(value) or value <= 0):
                raise ValueError(f'{key} must be finite and positive')
            self.settings[key] = value
        self.provenance = runtime_provenance(self.settings)
        self.store = SessionStore(self.settings['session_directory'])
        self.bridge = CvBridge()
        self._inputs = CaptureInputReceiver(self)
        self.buffer = self._inputs.buffer
        try:
            self.configure_topics({})
            self.commands = MutuallyExclusiveCallbackGroup()
            self.service = self.create_service(CalibrationCommand, '~/command', self.command,
                                               callback_group=self.commands)
        except Exception:
            self._inputs.close()
            super().destroy_node()
            raise

    def configure_topics(self, metadata):
        topics = tuple(metadata.get(key, self.settings[key]) for key in ('image_topic', 'camera_info_topic'))
        for topic in topics:
            if not isinstance(topic, str):
                raise ValueError('camera topics must be absolute ROS topic names')
            validate_full_topic_name(topic)
        self._inputs.configure(topics)
        effective = dict(self.settings, image_topic=topics[0], camera_info_topic=topics[1])
        self.provenance = runtime_provenance(effective)

    def destroy_node(self):
        self._inputs.close()
        return super().destroy_node()

    def observation(self, metadata, after_ns, checks=None):
        if checks:
            checks.start('image')
        snapshot = self._inputs.snapshot()
        image, infos = snapshot.image, snapshot.infos
        if image is None:
            raise WaitingObservation('waiting for Image')
        stamp = stamp_ns(image.header.stamp)
        age = (self.get_clock().now().nanoseconds - stamp) / 1e9
        if stamp <= after_ns:
            raise WaitingObservation('waiting for advancing image')
        if stamp <= 0 or not -.05 <= age <= self.settings['maximum_age_sec']:
            raise ValueError('waiting for fresh advancing image')
        if image.header.frame_id != metadata['camera_frame']:
            raise ValueError('optical frame mismatch')
        if checks:
            checks.passed(f'图像新鲜，帧龄 {age:.3f} 秒')
            checks.start('camera_info')
        if not infos:
            raise WaitingObservation('waiting for CameraInfo')
        info = min(infos, key=lambda i: abs(stamp_ns(i.header.stamp) - stamp))
        if abs(stamp_ns(info.header.stamp) - stamp) / 1e9 > self.settings['maximum_info_skew_sec']:
            raise ValueError('CameraInfo time mismatch')
        if image.header.frame_id != metadata['camera_frame'] or info.header.frame_id != metadata['camera_frame']:
            raise ValueError('optical frame mismatch')
        if (image.width, image.height) != (info.width, info.height):
            raise ValueError('CameraInfo size mismatch')
        if info.distortion_model not in ('plumb_bob', 'rational_polynomial'):
            raise ValueError('unsupported distortion model')
        if checks:
            checks.passed('时间、分辨率、坐标系与畸变模型匹配')
            checks.start('tf')
        tf = self.buffer.lookup_transform(metadata['base_frame'], metadata['end_link_frame'],
                                         rclpy.time.Time.from_msg(image.header.stamp))
        # Explicit historical lookup; no prior end->camera calibration is used.
        t, q = tf.transform.translation, tf.transform.rotation
        base_end = {'translation': [t.x, t.y, t.z], 'rotation_xyzw': [q.x, q.y, q.z, q.w]}
        transform_matrix(base_end)
        if checks:
            checks.passed('图像时刻的基座到末端 TF 可用')
            checks.start('aruco')
        marker = detect_aruco_pose(self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8'),
                 camera_matrix=np.asarray(info.k).reshape(3, 3), distortion=info.d,
                 marker_length_m=metadata['marker_length_m'],
                 dictionary_name=metadata.get('dictionary', 'DICT_4X4_50'),
                 marker_id=metadata.get('marker_id', 0))
        if marker['reprojection_rmse_px'] > self.settings['maximum_reprojection_px']:
            raise ValueError('reprojection error too large')
        if marker['area_px2'] < self.settings['minimum_area_px2']:
            raise ValueError('marker too small')
        if not 0 < np.linalg.norm(marker['camera_to_marker']['translation']) <= self.settings['maximum_distance_m']:
            raise ValueError('marker distance invalid')
        if checks:
            checks.passed(f"目标标记检出，重投影误差 {marker['reprojection_rmse_px']:.3f} px；面积与距离通过")
        return {'provenance': self.provenance, 'base_to_end': base_end, 'camera_to_marker': marker['camera_to_marker'],
                'image_stamp_ns': stamp, 'tf_stamp_ns': stamp_ns(tf.header.stamp),
                'capture_age_sec': age, 'monotonic_ns': time.monotonic_ns(),
                'quality': {k: marker[k] for k in ('area_px2', 'reprojection_rmse_px')},
                'camera_info': {'k': list(info.k), 'd': list(info.d), 'width': info.width,
                                'height': info.height, 'distortion_model': info.distortion_model}}

    def capture(self, metadata, checks=None):
        if metadata.get('mode') != 'tcp':
            self.configure_topics(metadata)
        deadline = time.monotonic() + self.settings['capture_timeout_sec']
        # Require observations acquired after this request, not a buffered window.
        after = self.get_clock().now().nanoseconds
        window = StabilityWindow(self.settings['stability_window_sec'], self.settings['maximum_sample_gap_sec'],
                                 self.settings['maximum_translation_m'], self.settings['maximum_rotation_deg'])
        reason = 'no observation'
        while rclpy.ok() and time.monotonic() < deadline:
            try:
                if checks:
                    checks.reset()
                if metadata.get('mode') == 'tcp':
                    if checks:
                        checks.start('tf')
                    checked = self.check_tf(metadata)
                    if checked['stamp_ns'] <= after:
                        raise WaitingObservation('waiting for advancing TF')
                    if checks:
                        checks.passed('新鲜基座到末端 TF 可用')
                    sample = {'base_to_end': checked['transform'], 'tf_stamp_ns': checked['stamp_ns'],
                              'capture_age_sec': checked['age_sec'], 'provenance': self.provenance,
                              'monotonic_ns': time.monotonic_ns()}
                else:
                    sample = self.observation(metadata, after, checks) if checks else self.observation(metadata, after)
                if checks:
                    checks.start('stability')
                after = sample['tf_stamp_ns']
                pose = transform_matrix(sample['base_to_end'])
                if window.add(after, pose):
                    sample['stability_sample_count'] = len(window.values)
                    sample['stability_duration_sec'] = (after - window.values[0][0]) / 1e9
                    if checks:
                        checks.passed(f"稳定窗口 {sample['stability_duration_sec']:.2f} 秒，{len(window.values)} 个观测")
                        sample['checks'] = checks.items
                    return sample
                reason = 'waiting for stable pose window'
            except (WaitingObservation, TransformException) as exc:
                reason = str(exc)
            except Exception as exc:
                window.reset()
                reason = str(exc)
            time.sleep(.02)
        if checks:
            from .preflight_checks import PreflightFailure
            checks.failed(reason)
            raise PreflightFailure(reason, checks)
        raise TimeoutError(reason)

    def check_tf(self, metadata):
        # Only base->end is a prerequisite: checking base->camera would require
        # the very hand-eye transform this session is intended to estimate.
        value = self.buffer.lookup_transform(metadata['base_frame'], metadata['end_link_frame'], rclpy.time.Time())
        stamp = stamp_ns(value.header.stamp)
        age = (self.get_clock().now().nanoseconds - stamp) / 1e9
        if stamp <= 0 or not -.05 <= age <= self.settings['maximum_age_sec']:
            raise ValueError('robot TF is stale or unstamped')
        t, q = value.transform.translation, value.transform.rotation
        pose = {'translation': [t.x, t.y, t.z], 'rotation_xyzw': [q.x, q.y, q.z, q.w]}
        transform_matrix(pose)
        return {'passed': True, 'target_frame': metadata['base_frame'],
                'source_frame': metadata['end_link_frame'], 'stamp_ns': stamp,
                'age_sec': age, 'transform': pose,
                'scope': 'latest base-to-end only; not a hand-eye accuracy check'}

    def preview(self):
        image = self._inputs.snapshot().image
        if image is None:
            raise ValueError('no camera image')
        age = (self.get_clock().now().nanoseconds - stamp_ns(image.header.stamp)) / 1e9
        if not -.05 <= age <= self.settings['maximum_age_sec']:
            raise ValueError('camera image is stale')
        pixels = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
        height, width = pixels.shape[:2]
        if width > 640:
            pixels = cv2.resize(pixels, (640, max(1, round(height * 640 / width))))
        ok, encoded = cv2.imencode('.jpg', pixels, [cv2.IMWRITE_JPEG_QUALITY, 75])
        if not ok:
            raise ValueError('preview encoding failed')
        return {'schema_version': 1, 'preview': {
            'jpeg_base64': base64.b64encode(encoded).decode('ascii'),
            'image_stamp_ns': stamp_ns(image.header.stamp), 'age_sec': age,
            'frame_id': image.header.frame_id}}

    def command(self, request, response):
        try:
            payload = json.loads(request.payload_json or '{}')
            if not isinstance(payload, dict):
                raise ValueError('payload must be object')
            if request.command == 'preview':
                if request.session_id:
                    metadata = self.store.read(request.session_id)['metadata']
                    self.configure_topics(metadata)
                data = self.preview()
            elif request.command == 'create':
                metadata = payload.get('metadata', {})
                length = metadata.get('marker_length_m', 0)
                if metadata.get('mode') != 'tcp' and (not isinstance(length, (float, int)) or not math.isfinite(length) or length <= 0):
                    raise ValueError('positive marker_length_m required')
                if metadata.get('mode') != 'tcp':
                    for key in ('image_topic', 'camera_info_topic'):
                        validate_full_topic_name(metadata.get(key, self.settings[key]))
                data = self.store.create(metadata, session_id=request.session_id)
                if metadata.get('mode') != 'tcp':
                    self.configure_topics(metadata)
            elif request.command == 'tf_check':
                data = self.store.read(request.session_id)
                data = dict(data, tf_check=self.check_tf(data['metadata']))
            elif request.command == 'status':
                data = self.store.read(request.session_id)
            elif request.command in ('capture', 'preflight'):
                data = self.store.read(request.session_id)
                if request.command == 'capture':
                    if set(payload) != {'split'} or payload['split'] not in ('training', 'validation'):
                        raise ValueError('capture accepts split only')
                    old = data.get('request_inputs', {}).get(request.request_id)
                    if old:
                        if old['operation'] != 'capture' or old['revision'] != request.expected_revision or old['payload']['split'] != payload['split']:
                            raise ValueError('request_id reused with different capture')
                        sample = old['payload']['sample']
                    else:
                        if data['state'] != 'active' or data['revision'] != request.expected_revision:
                            raise ValueError('capture requires active session and current revision')
                        sample = self.capture(data['metadata'])
                    data = self.store.mutate(request.session_id, request_id=request.request_id,
                        revision=request.expected_revision, operation='capture',
                        payload={'split': payload['split'], 'sample': sample})
                else:
                    from .preflight_checks import PreflightChecks
                    data = dict(data, preflight=self.capture(data['metadata'], PreflightChecks(data['metadata'].get('mode') == 'tcp')))
            else:
                data = self.store.mutate(request.session_id, request_id=request.request_id,
                    revision=request.expected_revision, operation=request.command, payload=payload)
            from .calibration_quality import sample_coverage
            if 'training_samples' in data:
                excluded = set(data.get('excluded_sample_ids', []))
                data = dict(data, coverage={split: sample_coverage([sample for sample in data[split + '_samples'] if sample['sample_id'] not in excluded], mode=data['metadata'].get('mode', 'handeye'))
                                           for split in ('training', 'validation')})
            data.setdefault('schema_version', 1)
            data.setdefault('session_id', request.session_id)
            data.setdefault('state', 'preview')
            response.session_json = json.dumps(data, allow_nan=False)
            response.success = True
            response.reason_code = 'OK'
            response.message = 'calibration command completed'
        except Exception as exc:
            response.success = False
            response.reason_code = 'CAPTURE_TIMEOUT' if isinstance(exc, TimeoutError) else 'COMMAND_REJECTED'
            response.message = str(exc)
            # Failure never claims a state transition. Read back only the actual
            # persisted revision, without echoing a partially modified object.
            failure = {'schema_version': 1, 'session_id': request.session_id,
                       'state': 'unavailable', 'revision': None}
            if request.command == 'preflight' and hasattr(exc, 'checks'):
                failure['preflight'] = {'checks': exc.checks}
            if request.session_id:
                try:
                    saved = self.store.read(request.session_id)
                    failure.update(state=saved['state'], revision=saved['revision'])
                except (OSError, ValueError, KeyError):
                    pass
            response.session_json = json.dumps(failure, allow_nan=False)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = HandeyeCaptureNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
