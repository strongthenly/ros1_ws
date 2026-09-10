#!/usr/bin/env python3
"""Require reviewed calibration and isolate unverified PX4 feedback."""
import argparse,os,subprocess,yaml
p=argparse.ArgumentParser();p.add_argument('--calibration',required=True);p.add_argument('--algorithm-config',required=True);p.add_argument('--camera-config',required=True);p.add_argument('--control-config')
a=p.parse_args()
with open(a.calibration) as f:c=yaml.safe_load(f) or {}
required=['algorithm_imu_source','lidar_imu_transform','lidar_camera_transform','body_lidar_transform','camera_intrinsics','sensor_time_offsets','imu_noise']
missing=[k for k in required if c.get(k) is None]
if c.get('status')!='verified' or not c.get('lidar_per_point_timing_verified') or missing:
    p.error('Calibration/timing not verified; missing: '+', '.join(missing))
for path in [a.algorithm_config,a.camera_config]:
    if not os.path.isfile(path):p.error('Config does not exist: '+path)
with open(a.algorithm_config) as f:algorithm=yaml.safe_load(f) or {}
with open(a.camera_config) as f:camera=yaml.safe_load(f) or {}
def nested(config,path):
    value=config
    for key in path.split('.'):
        value=value.get(key) if isinstance(value,dict) else None
    return value
algorithm_required=['common.img_topic','common.lid_topic','common.imu_topic','extrin_calib.extrinsic_T','extrin_calib.extrinsic_R','extrin_calib.Rcl','extrin_calib.Pcl','time_offset.imu_time_offset','time_offset.img_time_offset','preprocess.lidar_type','preprocess.scan_line','preprocess.blind','imu.acc_cov','imu.gyr_cov','imu.b_acc_cov','imu.b_gyr_cov']
camera_required=['cam_model','cam_width','cam_height','scale','cam_fx','cam_fy','cam_cx','cam_cy','cam_d0','cam_d1','cam_d2','cam_d3']
algorithm_missing=[key for key in algorithm_required if nested(algorithm,key) is None]
camera_missing=[key for key in camera_required if nested(camera,key) is None]
expected_topics={'common.img_topic':'/left_camera/image','common.lid_topic':'/livox/lidar','common.imu_topic':'/livox/imu'}
topic_errors=[key for key,value in expected_topics.items() if nested(algorithm,key)!=value]
if algorithm.get('twin_status')!='verified' or algorithm_missing or topic_errors:
    p.error('FAST-LIVO2 config not verified; missing: %s; wrong topics: %s' % (', '.join(algorithm_missing),', '.join(topic_errors)))
if camera.get('twin_status')!='verified' or camera_missing:
    p.error('D435 camera config not verified; missing: '+', '.join(camera_missing))
allow_mavros_feedback=False
if a.control_config:
    if not os.path.isfile(a.control_config):p.error('Control config does not exist: '+a.control_config)
    with open(a.control_config) as f:control=yaml.safe_load(f) or {}
    control_required=['px4_version','fcu_url','gcs_url','mavlink_system_id','mavlink_component_id']
    control_missing=[k for k in control_required if control.get(k) in (None,'')]
    if control.get('status')!='verified' or not control.get('failsafe_behavior_verified') or control_missing:
        p.error('PX4 feedback not verified; missing: '+', '.join(control_missing))
    allow_mavros_feedback=True
subprocess.check_call(['rosparam','load',a.algorithm_config])
subprocess.check_call(['rosparam','load',a.camera_config,'/laserMapping'])
command=['rosrun','fast_livo','fastlivo_mapping','__name:=laserMapping']
if not allow_mavros_feedback:
    command.append('/mavros/vision_pose/pose:=/twin/fast_livo/vision_pose_preview')
os.execvp('rosrun',command)
