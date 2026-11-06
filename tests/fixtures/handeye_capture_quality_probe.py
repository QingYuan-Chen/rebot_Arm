"""隔离真实 ROS 图像/TF：视觉抖动拒绝、稳定采集、证据与重复姿态准入。"""
import base64
import hashlib
import json
from pathlib import Path
import sys
import threading

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState
from tf2_ros import TransformBroadcaster
from rebotarm_msgs.srv import CalibrationCommand
from rebotarm_calibration.handeye_capture_node import HandeyeCaptureNode

rclpy.init(args=['--ros-args', '-p', 'capture_timeout_sec:=1.0', '-p', 'session_directory:='+sys.argv[1]])
node = HandeyeCaptureNode()
source = Node('handeye_quality_source')
bridge = CvBridge()
images = source.create_publisher(Image, '/camera/color/image_raw', 10)
infos = source.create_publisher(CameraInfo, '/camera/color/camera_info', 10)
joints = source.create_publisher(JointState, '/rebotarm/joint_states', qos_profile_sensor_data)
tf = TransformBroadcaster(source)
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
marker = (cv2.aruco.generateImageMarker(dictionary, 0, 90) if hasattr(cv2.aruco, 'generateImageMarker')
          else cv2.aruco.drawMarker(dictionary, 0, 90))
scene = {'jitter': True, 'sequence': 0, 'robot_x': .1}
def publish():
    scene['sequence'] += 1
    stamp = source.get_clock().now().to_msg()
    frame = np.full((480,640,3),255,np.uint8)
    x = 270+(14 if scene['sequence'] % 2 else -14) if scene['jitter'] else 270
    frame[180:270,x:x+90] = cv2.cvtColor(marker,cv2.COLOR_GRAY2BGR)
    info = CameraInfo()
    info.header.stamp, info.header.frame_id = stamp, 'optical'
    info.width, info.height = 640, 480
    info.k, info.d, info.distortion_model = [392.5,0.,318.75,0.,392.3,229.9,0.,0.,1.], [0.]*5, 'plumb_bob'
    image = bridge.cv2_to_imgmsg(frame,'bgr8')
    image.header = info.header
    joint = JointState()
    joint.header.stamp, joint.header.frame_id = stamp, 'base_link'
    joint.name, joint.position = [f'joint{i}' for i in range(1,7)], [0.]*6
    joints.publish(joint)
    transform = TransformStamped()
    transform.header.stamp, transform.header.frame_id, transform.child_frame_id = stamp, 'base', 'end'
    transform.transform.translation.x, transform.transform.rotation.w = scene['robot_x'], 1.
    tf.sendTransform(transform)
    infos.publish(info)
    images.publish(image)

source.create_timer(.05,publish)
executor = SingleThreadedExecutor()
executor.add_node(source)
thread = threading.Thread(target=executor.spin)
thread.start()
def call(command, request='', revision=0, payload=None):
    return node.command(CalibrationCommand.Request(command=command,session_id='a'*32,
        request_id=request,expected_revision=revision,payload_json=json.dumps(payload or {})),CalibrationCommand.Response())
try:
    metadata = dict(base_frame='base',end_link_frame='end',camera_frame='optical',
                    marker_frame='board',marker_length_m=.075)
    assert call('create',payload={'metadata':metadata}).success
    subscriptions = node._inputs.get_subscriptions_info_by_topic('/camera/color/image_raw')
    assert any(s.node_name==node._inputs.get_name() and s.qos_profile.reliability==ReliabilityPolicy.RELIABLE
               for s in subscriptions)
    rejected = call('capture','jitter',payload={'split':'training'})
    assert not rejected.success and rejected.reason_code=='CAPTURE_TIMEOUT',rejected
    assert node.store.read('a'*32)['revision']==0
    scene['jitter'] = False
    accepted = call('capture','steady',payload={'split':'training'})
    assert accepted.success,accepted
    saved = node.store.read('a'*32)
    sample = saved['training_samples'][0]
    assert sample['stability_duration_sec']>=.4
    window = sample['capture_window']
    observations = window['observations']
    assert len(observations)==sample['stability_sample_count']
    selected = observations[window['selected_index']]
    for key in ('image_stamp_ns','tf_stamp_ns','base_to_end','camera_to_marker','corners_px'):
        assert sample[key]==selected[key],key
    assert sample['image_stamp_ns']==sample['camera_info_stamp_ns']==sample['tf_stamp_ns']
    assert sample['joint_state_evidence']['samples']
    assert any(abs(j['offset_sec'])<.1 for j in sample['joint_state_evidence']['samples'])
    png = base64.b64decode(sample['image']['data_base64'])
    assert hashlib.sha256(png).hexdigest()==sample['image']['sha256']
    pixels = cv2.imdecode(np.frombuffer(png,np.uint8),cv2.IMREAD_COLOR)
    solved = node.marker_pose(metadata,bridge.cv2_to_imgmsg(pixels,'bgr8'),node._inputs.snapshot().infos[-1])
    assert np.allclose(solved['corners_px'],sample['corners_px'])
    path = Path(sys.argv[1])/('a'*32+'.json')
    raw = path.read_bytes()
    duplicate = call('capture','duplicate',revision=1,payload={'split':'validation'})
    assert not duplicate.success and 'duplicate robot pose matches sample steady' in duplicate.message
    assert path.read_bytes()==raw
    retry = call('capture','steady',payload={'split':'training'})
    assert retry.success and node.store.read('a'*32)==saved
    scene['robot_x'] = .12
    next_sample = call('capture','new-pose',revision=1,payload={'split':'validation'})
    assert next_sample.success,next_sample
    assert node.store.read('a'*32)['revision']==2
    print(json.dumps(dict(jitter='rejected',stable='accepted',duplicate='rejected',png_sha256=sample['image']['sha256'],
                          evidence_frames=len(observations))))
finally:
    executor.shutdown()
    thread.join()
    node.destroy_node()
    source.destroy_node()
    rclpy.shutdown()
