#!/usr/bin/env python3
"""Read-only preview bridge. No invented Livox timestamps or real calibration."""
import math, socket, threading, time
import msgpack
import numpy as np
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Imu, Image, CameraInfo
from sensor_msgs import point_cloud2
from rosgraph_msgs.msg import Clock

class Rpc:
    def __init__(self, host, port):
        self.host, self.port, self.sock, self.seq = host, port, None, 0
    def call(self, method, args):
        try:
            if self.sock is None:
                self.sock = socket.create_connection((self.host,self.port),3)
                self.sock.settimeout(3)
                self.unpacker=msgpack.Unpacker(raw=False, max_buffer_size=64*1024*1024)
            self.seq+=1
            self.sock.sendall(msgpack.packb([0,self.seq,method,args],use_bin_type=True))
            while True:
                for response in self.unpacker:
                    if response[0]!=1 or response[1]!=self.seq: raise RuntimeError('RPC response mismatch')
                    if response[2] is not None: raise RuntimeError(str(response[2]))
                    return response[3]
                chunk=self.sock.recv(65536)
                if not chunk: raise ConnectionError('AirSim disconnected')
                self.unpacker.feed(chunk)
        except Exception:
            if self.sock: self.sock.close()
            self.sock=None
            raise

def flu(v):
    return (float(v['x_val']),-float(v['y_val']),-float(v['z_val']))

class Bridge:
    def __init__(self):
        self.host=rospy.get_param('~host','127.0.0.1');self.port=int(rospy.get_param('~port',41451))
        self.vehicle=rospy.get_param('~vehicle','PX4')
        self.camera_name=rospy.get_param('~camera_name','front_center')
        self.lock=threading.Lock();self.clock_ns=0;self.seen={}
        self.clock=rospy.Publisher('/clock',Clock,queue_size=10)
        self.imu=rospy.Publisher('/twin/imu/data_raw',Imu,queue_size=200)
        self.lidar=rospy.Publisher('/twin/lidar/points_preview',point_cloud2.PointCloud2,queue_size=3)
        self.rgb=rospy.Publisher('/twin/camera/color/image_raw',Image,queue_size=2)
        self.depth=rospy.Publisher('/twin/camera/depth/image_raw',Image,queue_size=2)
        self.info=rospy.Publisher('/twin/camera/color/camera_info',CameraInfo,queue_size=2)
    def stamp(self,key,ns):
        ns=int(ns)
        if ns<=0: raise ValueError('Missing sensor timestamp')
        with self.lock:
            last=self.seen.get(key,0)
            if ns<last: raise RuntimeError('Simulation time moved backwards; restart bridge and estimator after reset')
            if ns==last:return None
            self.seen[key]=ns
            stamp=rospy.Time(ns//1000000000,ns%1000000000)
            if ns>self.clock_ns:
                self.clock_ns=ns;self.clock.publish(Clock(stamp))
            return stamp
    def read_imu(self,rpc):
        d=rpc.call('getImuData',['TwinImu',self.vehicle]);s=self.stamp('imu',d['time_stamp'])
        if s is None:return
        m=Imu();m.header=Header(stamp=s,frame_id='twin_imu_flu')
        m.orientation_covariance[0]=-1 # do not leak ground-truth attitude to the estimator
        m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z=flu(d['angular_velocity'])
        m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z=flu(d['linear_acceleration'])
        self.imu.publish(m)
    def read_lidar(self,rpc):
        d=rpc.call('getLidarData',['TwinLidarPreview',self.vehicle]);s=self.stamp('lidar',d['time_stamp'])
        if s is None:return
        p=np.asarray(d['point_cloud'],dtype=np.float32).reshape(-1,3)
        p=p[np.isfinite(p).all(axis=1)];p[:,1:]*=-1
        fields=[point_cloud2.PointField('x',0,point_cloud2.PointField.FLOAT32,1),point_cloud2.PointField('y',4,point_cloud2.PointField.FLOAT32,1),point_cloud2.PointField('z',8,point_cloud2.PointField.FLOAT32,1),point_cloud2.PointField('rgb',12,point_cloud2.PointField.FLOAT32,1)]
        pts=[[float(q[0]),float(q[1]),float(q[2]),0.0] for q in p]
        self.lidar.publish(point_cloud2.create_cloud(Header(stamp=s,frame_id='twin_lidar_flu'),fields,pts))
    def read_rgb(self,rpc):
        req=[dict(camera_name=self.camera_name,image_type=0,pixels_as_float=False,compress=False)]
        d=rpc.call('simGetImages',[req,self.vehicle,False])[0]
        w,h=int(d['width']),int(d['height'])
        if w<=0 or h<=0:raise ValueError('RGB camera returned empty image')
        s=self.stamp('rgb',d['time_stamp'])
        if s is None:return
        raw=d['image_data_uint8'];raw=raw.encode('latin1') if isinstance(raw,str) else bytes(raw)
        assert len(raw)==w*h*3, 'Unexpected RGB buffer length'
        m=Image();m.header=Header(stamp=s,frame_id='twin_camera_optical');m.width=w;m.height=h;m.is_bigendian=0
        m.encoding='rgb8';m.step=w*3;m.data=raw;self.rgb.publish(m)
        f=w/(2*math.tan(math.radians(float(rospy.get_param('~camera_fov_deg',90)))/2))
        info=CameraInfo();info.header=m.header;info.width=w;info.height=h;info.distortion_model='plumb_bob'
        info.D=[0.]*5;info.K=[f,0.,w/2,0.,f,h/2,0.,0.,1.];info.R=[1.,0.,0.,0.,1.,0.,0.,0.,1.]
        info.P=[f,0.,w/2,0.,0.,f,h/2,0.,0.,0.,1.,0.];self.info.publish(info)
    def read_depth(self,rpc):
        req=[dict(camera_name=self.camera_name,image_type=1,pixels_as_float=True,compress=False)]
        d=rpc.call('simGetImages',[req,self.vehicle,False])[0]
        w,h=int(d['width']),int(d['height'])
        if w<=0 or h<=0:raise ValueError('Depth camera returned empty image')
        s=self.stamp('depth',d['time_stamp'])
        if s is None:return
        data=np.asarray(d['image_data_float'],dtype='<f4');assert data.size==w*h
        m=Image();m.header=Header(stamp=s,frame_id='twin_camera_optical');m.width=w;m.height=h;m.is_bigendian=0
        m.encoding='32FC1';m.step=w*4;m.data=data.tobytes();self.depth.publish(m)
    def loop(self,method,rate):
        rpc=Rpc(self.host,self.port)
        while not rospy.is_shutdown():
            start=time.monotonic()
            try:method(rpc)
            except Exception as e:rospy.logwarn_throttle(5,'%s: %s',method.__name__,e)
            time.sleep(max(0.001,1/rate-(time.monotonic()-start)))
    def run(self):
        for name in ('imu','lidar','rgb','depth'):
            rate=float(rospy.get_param('~'+name+'_rate',10))
            if rate<=0:raise ValueError('Polling rate must be positive')
            threading.Thread(target=self.loop,args=(getattr(self,'read_'+name),rate),daemon=True).start()
        rospy.spin()
if __name__=='__main__':
    rospy.init_node('twin_sensor_bridge');Bridge().run()
