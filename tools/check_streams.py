#!/usr/bin/env python3
import time,json,threading
import rospy
from sensor_msgs.msg import Imu,Image,PointCloud2,CameraInfo
def main():
    rospy.init_node('twin_stream_check',anonymous=True)
    duration=float(rospy.get_param('~duration',10));stats={};lock=threading.Lock()
    topics={'/twin/imu/data_raw':Imu,'/twin/lidar/points_preview':PointCloud2,'/twin/camera/color/image_raw':Image,'/twin/camera/depth/image_raw':Image,'/twin/camera/color/camera_info':CameraInfo}
    def cb(msg,topic):
        with lock:
            s=stats.setdefault(topic,dict(count=0,non_increasing=0,first_ns=0,last_ns=0,frame_id=''))
            ns=msg.header.stamp.to_nsec()
            if ns<=s['last_ns']:s['non_increasing']+=1
            if s['count']==0:s['first_ns']=ns
            s.update(count=s['count']+1,last_ns=ns,frame_id=msg.header.frame_id)
            if isinstance(msg,PointCloud2):
                points=int(msg.width)*int(msg.height)
                s['point_frames']=s.get('point_frames',0)+1
                s['point_total']=s.get('point_total',0)+points
                s['points_min']=min(s.get('points_min',points),points)
                s['points_max']=max(s.get('points_max',points),points)
    subscriptions=[rospy.Subscriber(t,c,cb,callback_args=t,queue_size=200) for t,c in topics.items()]
    time.sleep(duration)
    with lock:
        result={}
        for topic in topics:
            s=dict(stats.get(topic,{'count':0,'non_increasing':0,'first_ns':0,'last_ns':0,'frame_id':''}))
            sim_span=(s['last_ns']-s['first_ns'])/1e9 if s['count']>1 else 0.0
            s['wall_hz']=s['count']/duration
            s['sim_span_s']=sim_span
            s['sim_hz']=(s['count']-1)/sim_span if sim_span>0 else 0.0
            s['realtime_factor']=sim_span/duration
            if s.get('point_frames',0)>0:
                s['points_average']=round(float(s['point_total'])/s['point_frames'],1)
            result[topic]=s
    print(json.dumps(result,indent=2))
    return 0 if all(s['count']>0 and s.get('non_increasing',0)==0 for s in result.values()) else 1
if __name__=='__main__':raise SystemExit(main())
