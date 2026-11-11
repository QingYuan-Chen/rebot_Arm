"""隔离状态桥与真实 robot_state_publisher,对照独立 Pinocchio FK 和历史时间."""
import json
from pathlib import Path
import subprocess
import sys
import threading
import time
import xml.etree.ElementTree as ET

import numpy as np
import pinocchio as pin
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener
import yaml

from rebotarm_calibration.handeye_residual import transform_matrix
from rebotarm_teleop.gripper_visual_joint_state_node import GripperVisualJointStateNode

root = Path(__file__).resolve().parents[2]
directory = Path(sys.argv[1])
description = (root/'src/rebotarm_moveit_config/config/rebotarm.urdf').read_text()
parameters = directory/'rsp.yaml'
parameters.write_text(yaml.safe_dump({'/**':{'ros__parameters':{
    'robot_description':description,'publish_frequency':100.}}}))
process = subprocess.Popen(['/opt/ros/jazzy/lib/robot_state_publisher/robot_state_publisher',
    '--ros-args','--params-file',str(parameters),'-r','/joint_states:=/tf_probe/visual_joint_states'],
    stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
rclpy.init(args=['--ros-args','-p','arm_namespace:=tf_probe'])
source = Node('tf_probe_source')
bridge = GripperVisualJointStateNode()
buffer = Buffer()
listener = TransformListener(buffer,source)
publisher = source.create_publisher(JointState,'/tf_probe/joint_states',qos_profile_sensor_data)
world = StaticTransformBroadcaster(source)
fixed = TransformStamped()
fixed.header.stamp = source.get_clock().now().to_msg()
fixed.header.frame_id, fixed.child_frame_id = 'world', 'base_link'
fixed.transform.translation.x, fixed.transform.translation.y = .21, -.08
fixed.transform.rotation.w = 1.
world.sendTransform(fixed)
world_base = np.eye(4)
world_base[:3,3] = [.21,-.08,0.]
executor = MultiThreadedExecutor(num_threads=2)
for node in (source,bridge):
    executor.add_node(node)
worker = threading.Thread(target=executor.spin)
worker.start()
model = pin.buildModelFromXML(description)
data = model.createData()
end = model.getFrameId('end_link')
def matrix(tf):
    t,q = tf.transform.translation,tf.transform.rotation
    return transform_matrix({'translation':[t.x,t.y,t.z],'rotation_xyzw':[q.x,q.y,q.z,q.w]})
def wait_for(test):
    end_time = time.monotonic()+4
    while not test():
        assert time.monotonic()<end_time,'TF probe timed out'
        time.sleep(.01)
poses = [np.zeros(6),np.array([0.,-.8,-1.2,.6,0.,0.])]
poses.extend(np.random.default_rng(22).uniform(-.7,.7,(10,6)))
records = []
try:
    wait_for(lambda:publisher.get_subscription_count()>0)
    history = None
    for pose in poses:
        message = JointState()
        message.header.stamp = source.get_clock().now().to_msg()
        message.name, message.position = [f'joint{i}' for i in range(1,7)],pose.tolist()
        message.header.frame_id = 'base_link'
        # 每个姿态采用独立源时间,重复发送同一帧仅等待 DDS 发现与交付.
        timestamp = Time.from_msg(message.header.stamp)
        def delivered():
            publisher.publish(message)
            return buffer.can_transform('base_link','end_link',timestamp)
        wait_for(delivered)
        positions = pin.neutral(model)
        for index in range(6):
            positions[model.joints[model.getJointId(f'joint{index+1}')].idx_q] = pose[index]
        pin.framesForwardKinematics(model,data,positions)
        expected = data.oMf[end].homogeneous.copy()
        actual = matrix(buffer.lookup_transform('base_link','end_link',timestamp))
        reverse = matrix(buffer.lookup_transform('end_link','base_link',timestamp))
        world_actual = matrix(buffer.lookup_transform('world','end_link',timestamp))
        assert np.allclose(actual,expected,atol=1e-10)
        assert np.allclose(reverse@actual,np.eye(4),atol=1e-10)
        assert np.allclose(world_actual,world_base@expected,atol=1e-10)
        assert buffer.lookup_transform('base_link','end_link',timestamp).header.stamp==message.header.stamp
        records.append(dict(joints=pose.tolist(),stamp_ns=timestamp.nanoseconds,
                            matrix_max_error=float(np.max(np.abs(actual-expected)))))
        if history is None:
            history = (timestamp,expected)
        time.sleep(.025)
    assert np.allclose(matrix(buffer.lookup_transform('base_link','end_link',history[0])),history[1],atol=1e-10)
    tree = ET.fromstring(description)
    end_joint = tree.find("joint[@name='end_joint']")
    assert end_joint.find('parent').attrib['link']=='link6'
    assert end_joint.find('child').attrib['link']=='end_link'
    print(json.dumps(dict(poses=len(records),root_frame='base_link',tip_frame='end_link',
        max_matrix_error=max(row['matrix_max_error'] for row in records),
        historical_lookup='passed',records=records)))
finally:
    executor.shutdown()
    worker.join()
    listener.unregister()
    bridge.destroy_node()
    source.destroy_node()
    rclpy.shutdown()
    process.terminate()
    output,_ = process.communicate(timeout=5)
    assert process.returncode==0,output
