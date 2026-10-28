"""Subprocess isolates real ROS bindings from legacy suite stubs."""
import os
from pathlib import Path
import subprocess
import sys


def test_real_node_service_without_camera_fails_closed(tmp_path):
    script = '''
import json
import time
import rclpy
from rclpy.parameter import Parameter
from rebotarm_msgs.srv import CalibrationCommand
from rebotarm_calibration.handeye_capture_node import HandeyeCaptureNode
rclpy.init(args=['--ros-args', '-p', 'capture_timeout_sec:=0.1', '-p', 'session_directory:=' + __import__('sys').argv[1]])
node = HandeyeCaptureNode()
try:
    request = CalibrationCommand.Request(command='create', session_id='a'*32,
        payload_json=json.dumps({'metadata': dict(base_frame='base', end_link_frame='end', camera_frame='optical', marker_frame='marker', marker_length_m=.1)}))
    response = node.command(request, CalibrationCommand.Response())
    assert response.success, response.message
    request.command = 'capture'
    request.request_id = 'one'
    request.payload_json = json.dumps({'split':'training'})
    started = time.monotonic()
    response = node.command(request, CalibrationCommand.Response())
    assert not response.success and response.reason_code == 'CAPTURE_TIMEOUT', response
    assert time.monotonic() - started < 2
    assert node.store.read('a'*32)['revision'] == 0
    failure=json.loads(response.session_json)
    assert failure==dict(schema_version=1,session_id='a'*32,state='active',revision=0)
    request.payload_json = json.dumps({'split':'training', 'sample':{}})
    response = node.command(request, CalibrationCommand.Response())
    assert not response.success and response.reason_code == 'COMMAND_REJECTED'
finally:
    node.destroy_node()
    rclpy.shutdown()
'''
    env = dict(os.environ, ROS_DOMAIN_ID='176')
    root = Path(__file__).resolve().parents[1]
    env['PYTHONPATH'] = str(root / 'src/rebotarm_calibration') + os.pathsep + env.get('PYTHONPATH', '')
    result = subprocess.run([sys.executable, '-c', script, str(tmp_path)], env=env,
                            capture_output=True, text=True, timeout=15)
    assert result.returncode == 0, result.stdout + result.stderr


def test_positive_ros_capture_and_retry(tmp_path):
    script = r'''
import json, threading, time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from rebotarm_msgs.srv import CalibrationCommand
from rebotarm_calibration.handeye_capture_node import HandeyeCaptureNode
rclpy.init(args=['--ros-args','-p','port:=0','-p','session_directory:='+__import__('sys').argv[1]])
node=HandeyeCaptureNode(); source=Node('synthetic_calibration_source')
from rebotarm_dashboard.teleop_status_panel_node import TeleopStatusPanelNode
panel=TeleopStatusPanelNode()
images=source.create_publisher(Image,'/camera/color/image_raw',qos_profile_sensor_data)
infos=source.create_publisher(CameraInfo,'/camera/color/camera_info',qos_profile_sensor_data)
alternate_images=source.create_publisher(Image,'/alternate/color',qos_profile_sensor_data)
alternate_infos=source.create_publisher(CameraInfo,'/alternate/info',qos_profile_sensor_data)
tf=TransformBroadcaster(source)
bridge=CvBridge(); image=np.full((480,640,3),255,np.uint8)
d=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
marker=cv2.aruco.drawMarker(d,0,240) if not hasattr(cv2.aruco,'generateImageMarker') else cv2.aruco.generateImageMarker(d,0,240)
image[120:360,200:440]=cv2.cvtColor(marker,cv2.COLOR_GRAY2BGR)
scene_lock=threading.Lock()
camera_enabled=True
robot_pose={'translation':[0.,0.,0.], 'rotation_xyzw':[0.,0.,0.,1.]}
def publish():
    stamp=source.get_clock().now().to_msg()
    t=TransformStamped();t.header.stamp=stamp;t.header.frame_id='base';t.child_frame_id='end';
    with scene_lock:
        pose=robot_pose; current_image=image
    t.transform.translation.x,t.transform.translation.y,t.transform.translation.z=pose['translation']
    t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w=pose['rotation_xyzw']
    tf.sendTransform(t)
    if not camera_enabled:return
    info=CameraInfo();info.header.stamp=stamp;info.header.frame_id='optical';info.width=640;info.height=480
    info.k=[520.,0.,320.,0.,520.,240.,0.,0.,1.];info.d=[0.]*5;info.distortion_model='plumb_bob'
    msg=bridge.cv2_to_imgmsg(current_image,'bgr8');msg.header=info.header
    infos.publish(info);images.publish(msg)
    alternate_infos.publish(info);alternate_images.publish(msg)
source.create_timer(.04,publish)
executor=MultiThreadedExecutor(num_threads=4);executor.add_node(node);executor.add_node(source);executor.add_node(panel)
thread=threading.Thread(target=executor.spin);thread.start()
client=source.create_client(CalibrationCommand,'/rebotarm_handeye_capture/command')
from urllib.request import Request, urlopen
from urllib.error import HTTPError
http=panel._server
def call(command,payload,request_id='',session_id='b'*32,revision=0):
    body=dict(command=command,session_id=session_id,request_id=request_id,expected_revision=revision,payload=payload)
    request=Request(f'http://127.0.0.1:{http.server_port}/api/calibration/command',data=json.dumps(body).encode(),headers={'Content-Type':'application/json'})
    with urlopen(request,timeout=10) as response:
        result=json.load(response)
        assert response.status==200 and result['success'],result
        return result['session']

try:
    assert client.wait_for_service(timeout_sec=3)
    with urlopen(f'http://127.0.0.1:{http.server_port}/calibration') as page:
        assert 'gravity-start' in page.read().decode()
    blocked=Request(f'http://127.0.0.1:{http.server_port}/api/calibration/gravity',
        data=json.dumps({'command':'start','confirmed':True}).encode(),headers={'Content-Type':'application/json'})
    try:
        urlopen(blocked)
        raise AssertionError('software dashboard must reject hardware mode change')
    except HTTPError as error:
        assert error.code==400 and not json.load(error)['accepted']
    call('create',{'metadata':dict(base_frame='base',end_link_frame='end',camera_frame='optical',marker_frame='marker',marker_length_m=.1)})
    data=call('capture',{'split':'training'},'capture-one')
    assert data['revision']==1
    sample=data['training_samples'][0]
    assert sample['provenance']['capture_settings']['maximum_age_sec']==.5
    assert len(sample['provenance']['source_sha256']['handeye_capture_node.py'])==64
    assert sample['image_stamp_ns']==sample['tf_stamp_ns']
    assert sample['quality']['reprojection_rmse_px']<1.
    assert abs(sample['camera_to_marker']['translation'][2]-.217)<.01
    assert call('capture',{'split':'training'},'capture-one')['revision']==1
    tf_check=call('tf_check',{})['tf_check']
    assert tf_check['passed'] and tf_check['target_frame']=='base'
    assert tf_check['source_frame']=='end' and tf_check['age_sec']<.5
    preview=call('preview',{})['preview']
    import base64
    decoded=cv2.imdecode(np.frombuffer(base64.b64decode(preview['jpeg_base64']),np.uint8),cv2.IMREAD_COLOR)
    assert decoded.shape==(480,640,3)
    assert preview['frame_id']=='optical'
    alternate_id='9'*32
    call('create',{'metadata':dict(base_frame='base',end_link_frame='end',camera_frame='optical',marker_frame='marker',marker_length_m=.1,
                                 image_topic='/alternate/color',camera_info_topic='/alternate/info')},session_id=alternate_id)
    alternate=call('capture',{'split':'training'},'alternate-one',session_id=alternate_id)
    assert alternate['training_samples'][0]['provenance']['capture_settings']['image_topic']=='/alternate/color'
    assert node._inputs.topics==('/alternate/color','/alternate/info')
    camera_enabled=False
    with node._inputs.lock:
        node._inputs.frames.clear();node._inputs.infos.clear()
    tcp_id='c'*32
    call('create',{'metadata':dict(mode='tcp',base_frame='base',end_link_frame='end')},session_id=tcp_id)
    tcp=call('capture',{'split':'training'},'tcp-one',session_id=tcp_id)
    assert tcp['revision']==1 and tcp['metadata']['mode']=='tcp'
    tcp_sample=tcp['training_samples'][0]
    assert 'camera_to_marker' not in tcp_sample and tcp_sample['tf_stamp_ns']>0
    assert tcp['coverage']['training']['mode']=='tcp'
    with urlopen(f'http://127.0.0.1:{http.server_port}/api/calibration/export?session_id='+tcp_id) as exported:
        assert exported.headers['Content-Disposition'].startswith('attachment;')
        saved=json.load(exported)
        assert saved['training_samples']==tcp['training_samples']
        assert saved['revision']==1
    # Complete a second TCP session using only published TF; never inject samples.
    from rebotarm_calibration.handeye_residual import matrix_transform
    complete_id='d'*32
    complete=call('create',{'metadata':dict(mode='tcp',base_frame='base',end_link_frame='end')},session_id=complete_id)
    pivot=np.array([.3,-.2,.25]);offset=np.array([-.04,.003,.002]);rng=np.random.default_rng(81)
    for index in range(12):
        rotation=cv2.Rodrigues(rng.uniform(-.6,.6,3))[0]
        pose=np.eye(4);pose[:3,:3]=rotation;pose[:3,3]=pivot-rotation@offset
        robot_pose=matrix_transform(pose)
        complete=call('capture',{'split':'training' if index<6 else 'validation'},
                      'pose-'+str(index),complete_id,complete['revision'])
    complete=call('solve',{},'solve-all',complete_id,complete['revision'])
    assert complete['report']['passed'],complete['report']
    assert np.allclose(complete['report']['tcp_offset_xyz'],offset,atol=1e-6)
    complete=call('accept',{'confirmed':True,'operator':'integration-test'},'accept-all',complete_id,complete['revision'])
    assert complete['state']=='accepted' and complete['report']['deployed'] is False
    # Re-open storage instance to prove recovery from the persisted file.
    from rebotarm_calibration.session_store import SessionStore
    restored=SessionStore(node.settings['session_directory']).read(complete_id)
    assert restored['report']==complete['report']
    with urlopen(f'http://127.0.0.1:{http.server_port}/api/calibration/export?session_id='+complete_id) as exported:
        final=json.load(exported)
        assert final['state']=='accepted' and final['revision']==14
        assert len(final['training_samples'])==len(final['validation_samples'])==6
    # Perspective-rendered ArUco observations, paired to a fixed marker and handeye.
    handeye=np.eye(4);handeye[:3,3]=[-.08,.01,.04]
    base_marker=np.eye(4);base_marker[:3,3]=[.3,-.2,.2]
    k=np.array([[520.,0.,320.],[0.,520.,240.],[0.,0.,1.]])
    corners=np.array([[-.05,.05,0],[.05,.05,0],[.05,-.05,0],[-.05,-.05,0]])
    source_corners=np.array([[0,0],[239,0],[239,239],[0,239]],np.float32)
    hand_id='e'*32
    hand=call('create',{'metadata':dict(base_frame='base',end_link_frame='end',camera_frame='optical',marker_frame='marker',marker_length_m=.1)},session_id=hand_id)
    for index in range(16):
        cam_marker=np.eye(4)
        cam_marker[:3,:3]=cv2.Rodrigues(rng.uniform(-.45,.45,3))[0]@np.diag([1.,-1.,-1.])
        cam_marker[:3,3]=[rng.uniform(-.035,.035),rng.uniform(-.035,.035),rng.uniform(.26,.4)]
        rvec=cv2.Rodrigues(cam_marker[:3,:3])[0]
        projected=cv2.projectPoints(corners,rvec,cam_marker[:3,3],k,np.zeros(5))[0].reshape(4,2).astype(np.float32)
        homography=cv2.getPerspectiveTransform(source_corners,projected)
        rendered=cv2.warpPerspective(marker,homography,(640,480),flags=cv2.INTER_NEAREST,borderValue=255)
        pose=base_marker@np.linalg.inv(cam_marker)@np.linalg.inv(handeye)
        with scene_lock:
            image=cv2.cvtColor(rendered,cv2.COLOR_GRAY2BGR);robot_pose=matrix_transform(pose)
        camera_enabled=True
        hand=call('capture',{'split':'training' if index<10 else 'validation'},'image-'+str(index),hand_id,hand['revision'])
    hand=call('solve',{},'solve-images',hand_id,hand['revision'])
    assert hand['report']['passed'],hand['report']
    selected=hand['report']['methods'][hand['report']['selected_method']]
    assert np.linalg.norm(np.array(selected['end_to_camera']['translation'])-handeye[:3,3])<.005
    hand=call('accept',{'confirmed':True,'operator':'rendered-integration'},'accept-images',hand_id,hand['revision'])
    assert hand['report']['accepted'] and not hand['report']['deployed']




finally:
    panel.destroy_node()
    executor.shutdown();thread.join();node.destroy_node();source.destroy_node();rclpy.shutdown()
'''
    env = dict(os.environ, ROS_DOMAIN_ID='177')
    root = Path(__file__).resolve().parents[1]
    env['PYTHONPATH'] = str(root / 'src/rebotarm_calibration') + os.pathsep + env.get('PYTHONPATH', '')
    result = subprocess.run([sys.executable, '-c', script, str(tmp_path)], env=env,
                            capture_output=True, text=True, timeout=65)
    assert result.returncode == 0, result.stdout + result.stderr
