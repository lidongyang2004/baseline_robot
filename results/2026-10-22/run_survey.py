#!/usr/bin/env python3
"""Reproduce this run after launching the simulator; stops control after survey."""
import argparse
import json
import time
from collections import defaultdict
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import TwistWithCovarianceStamped
from std_msgs.msg import String
from std_srvs.srv import SetBool
from mvp_msgs.srv import ChangeState
from tf2_ros import Buffer, TransformListener

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output-dir', type=Path, required=True,
                    help='Save the new mission.json here; existing results are protected')
output=parser.parse_args().output_dir.expanduser().resolve()
output.mkdir(parents=True, exist_ok=True)
if (output/'mission.json').exists():
    parser.error('mission.json already exists; choose a new output directory')
rclpy.init()
node=Node('navigation_validation')
received=defaultdict(list)
state={'name':None,'seen_survey':False,'completed_at':None}
def odom(key,msg):
    p=msg.pose.pose.position
    received[key].append((time.monotonic(),msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,p.x,p.y,p.z))
def helm(msg):
    if state['name']!=msg.data: print('HELM',msg.data,flush=True)
    state['name']=msg.data
    if msg.data=='survey':state['seen_survey']=True
    if state['seen_survey'] and msg.data=='start' and state['completed_at'] is None:
        state['completed_at']=time.monotonic()
subs=[node.create_subscription(Odometry,'/baseline_robot/'+topic,lambda m,k=key:odom(k,m),qos_profile_sensor_data) for key,topic in [('filtered','odometry/filtered'),('truth','gps/ground_truth/odometry'),('depth','depth')]]
subs.append(node.create_subscription(NavSatFix,'/baseline_robot/gps/ground_truth/fix',lambda m:received['fix'].append(time.monotonic()),qos_profile_sensor_data))
subs.append(node.create_subscription(TwistWithCovarianceStamped,'/baseline_robot/dvl/twist',lambda m:received['dvl'].append(time.monotonic()),qos_profile_sensor_data))
subs.append(node.create_subscription(Imu,'/baseline_robot/imu/data',lambda m:received['imu'].append(time.monotonic()),qos_profile_sensor_data))
subs.append(node.create_subscription(String,'/baseline_robot/mvp_helm/current_helm_state',helm,10))
controller=node.create_client(SetBool,'/baseline_robot/controller/set')
mission=node.create_client(ChangeState,'/baseline_robot/mvp_helm/change_state')
buffer=Buffer(); listener=TransformListener(buffer,node)
def spin_for(seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
def call(client,request):
    future=client.call_async(request)
    rclpy.spin_until_future_complete(node,future,timeout_sec=5)
    if not future.done():raise RuntimeError('Service timed out')
    result=future.result(); print('SERVICE',result,flush=True); return result
try:
    end=time.monotonic()+20
    while time.monotonic()<end:
        rclpy.spin_once(node,timeout_sec=.05)
        if controller.service_is_ready() and mission.service_is_ready() and all(received[k] for k in ('truth','filtered','depth','dvl')):break
    else:raise RuntimeError('Required streams/services missing')
    spin_for(8)
    tf=buffer.lookup_transform('baseline_robot/base_link','baseline_robot/dvl_sf',rclpy.time.Time())
    assert abs(tf.transform.translation.x+.55)<1e-10
    assert abs(tf.transform.translation.z+.1)<1e-10
    assert call(controller,SetBool.Request(data=True)).success
    req=ChangeState.Request();req.state='survey';req.caller='navigation_validation'
    assert call(mission,req).status
    started=time.monotonic(); started_stamp=time.time(); last_progress=started
    while time.monotonic()-started<240:
        rclpy.spin_once(node,timeout_sec=.05)
        now=time.monotonic()
        if now-last_progress>10:
            print('PROGRESS',round(now-started,1),state['name'],received['filtered'][-1][2:],received['truth'][-1][2:],flush=True)
            last_progress=now
        if state['completed_at'] and now-state['completed_at']>=12:break
    else:raise RuntimeError('Full rectangle did not complete within 240 seconds')
    assert call(controller,SetBool.Request(data=False)).success
    truth=received['truth']
    assert max(v[2] for v in truth)>15 and max(v[3] for v in truth)>15
    summary={'final_state':state['name'],'mission_start_stamp':started_stamp,'mission_duration_sec':state['completed_at']-started,'post_mission_observation_sec':12,'mission_elapsed':time.monotonic()-started,'samples':{},'rates_hz':{}}
    for key,rows in received.items():
        summary['samples'][key]=len(rows)
        times=[r[0] for r in rows] if key in ('filtered','truth','depth') else rows
        summary['rates_hz'][key]=(len(times)-1)/(times[-1]-times[0])
    (output/'mission.json').write_text(json.dumps(summary,indent=2))
    print('COMPLETE',json.dumps(summary),flush=True)
finally:
    node.destroy_node();rclpy.shutdown()
