"""Actual Dashboard HTTP and ROS clients; isolated controller services only."""
import os
import subprocess
import sys


def test_gravity_mode_services_with_fake_controller():
    script = r'''
import json,time,threading
from urllib.request import Request,urlopen
from urllib.error import HTTPError
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from std_srvs.srv import Trigger
from rebotarm_msgs.msg import ArmStatus
from rebotarm_dashboard.teleop_status_panel_node import TeleopStatusPanelNode
rclpy.init(args=['--ros-args','-p','port:=0','-p','use_hardware:=true','-p','web_execute_enabled:=true','-p','execution_mode:=execute','-p','arm_namespace:=calibration_test'])
panel=TeleopStatusPanelNode(); controller=Node('fake_calibration_controller')
state={'enabled':False,'mode':'IDLE','stale':True,'errors':[]};calls=[]
def handler(name):
    def call(request,response):
        calls.append(name)
        if name=='enable':state['enabled']=True
        if name=='start':state['mode']='GRAVITY_COMP'
        if name=='stop':state['mode']='IDLE';state['errors']=[]
        response.success=True;response.message='fake '+name;return response
    return call
for name,path in [('enable','enable'),('start','gravity_compensation/start'),('stop','gravity_compensation/stop')]:
    controller.create_service(Trigger,'/calibration_test/'+path,handler(name))
qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
publisher=controller.create_publisher(ArmStatus,'/calibration_test/arm_status',qos)
def publish():
    message=ArmStatus();message.header.stamp=controller.get_clock().now().to_msg()
    if state['stale']:message.header.stamp.sec-=10
    message.enabled=state['enabled'];message.state_machine=state['mode'];message.mode='pos_vel'
    message.error_codes=state['errors']
    publisher.publish(message)
controller.create_timer(.03,publish)
executor=MultiThreadedExecutor(num_threads=3);executor.add_node(panel);executor.add_node(controller)
thread=threading.Thread(target=executor.spin);thread.start()
def wait(predicate):
    end=time.monotonic()+4
    while not predicate() and time.monotonic()<end:time.sleep(.02)
    assert predicate()
def post(path,payload):
    request=Request(f'http://127.0.0.1:{panel._server.server_port}'+path,data=json.dumps(payload).encode(),headers={'Content-Type':'application/json'})
    try:
        with urlopen(request,timeout=6) as result:return json.load(result)
    except HTTPError as error:return json.load(error)
try:
    wait(lambda:panel._store.snapshot().arm['state_machine']=='IDLE')
    assert panel._arm_status_received_at is None
    assert not post('/api/calibration/gravity',{'command':'start','confirmed':True})['accepted']
    assert calls==[]
    state['stale']=False
    wait(lambda:panel._arm_status_received_at is not None and panel._gravity_start_client.service_is_ready() and panel._arm_enable_client.service_is_ready())
    assert not post('/api/calibration/gravity',{'command':'start','confirmed':True})['accepted']
    assert calls==[]
    assert post('/api/arm_enable',{})['accepted']
    wait(lambda:panel._store.snapshot().arm['enabled'])
    assert not post('/api/calibration/gravity',{'command':'start','confirmed':False})['accepted']
    assert post('/api/calibration/gravity',{'command':'start','confirmed':True})['accepted']
    wait(lambda:panel._store.snapshot().arm['state_machine']=='GRAVITY_COMP')
    assert not post('/api/keyboard_enable',{})['accepted']
    assert calls==['enable','start']
    wait(lambda:panel._store.snapshot().teleop['gravity_comp']['active'])
    state['errors']=['GRAVITY_HOLD: three consecutive control deadline misses']
    wait(lambda:panel._store.snapshot().teleop['gravity_comp']['state']=='fault_hold')
    gravity=panel._store.snapshot().teleop['gravity_comp']
    assert not gravity['active'] and not gravity['ready_for_teach_recording']
    assert 'GRAVITY_HOLD' in gravity['message']
    assert not post('/api/calibration/gravity',{'command':'start','confirmed':True})['accepted']
    assert calls==['enable','start']
    assert post('/api/calibration/gravity',{'command':'stop','confirmed':True})['accepted']
    wait(lambda:panel._store.snapshot().arm['state_machine']=='IDLE' and panel._arm_status_received_at>panel._calibration_gravity_tracker.completed_at)
    assert panel._calibration_gravity_owned
    assert post('/api/calibration/gravity',{'command':'stop','confirmed':True})['accepted']
    assert not panel._calibration_gravity_owned
    assert calls==['enable','start','stop']
finally:
    panel.destroy_node();executor.shutdown();thread.join();controller.destroy_node();rclpy.shutdown()
'''
    result=subprocess.run([sys.executable,'-c',script],env=dict(os.environ,ROS_DOMAIN_ID='181'),
                          capture_output=True,text=True,timeout=25)
    assert result.returncode==0,result.stdout+result.stderr
