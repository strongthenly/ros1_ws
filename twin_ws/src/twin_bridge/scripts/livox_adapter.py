#!/usr/bin/env python3
"""Strict conversion of timed local FLU point clouds; rejects plain AirSim XYZ."""
import math
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from livox_ros_driver.msg import CustomMsg,CustomPoint
FIELDS=('x','y','z','offset_time','reflectivity','tag','line')
def convert(msg):
    if not set(FIELDS).issubset({f.name for f in msg.fields}):
        raise ValueError('Timed cloud requires x,y,z,offset_time(ns),reflectivity,tag,line; plain XYZ is not sufficient')
    if msg.header.stamp.to_nsec()<=0 or not msg.header.frame_id:raise ValueError('Missing frame or scan-start timestamp')
    out=CustomMsg();out.header=msg.header;out.timebase=msg.header.stamp.to_nsec();out.lidar_id=0
    rows=list(point_cloud2.read_points(msg,field_names=FIELDS,skip_nans=False))
    rows.sort(key=lambda r:r[3])
    for x,y,z,dt,reflectivity,tag,line in rows:
        if not all(math.isfinite(v) for v in [x,y,z,dt]):raise ValueError('Nonfinite point')
        if not 0<=dt<=1000000000 or int(dt)!=dt:raise ValueError('Invalid point offset in ns')
        if not all(0<=v<=255 and int(v)==v for v in [reflectivity,tag,line]):raise ValueError('Invalid Livox metadata')
        p=CustomPoint();p.x=x;p.y=y;p.z=z;p.offset_time=int(dt);p.reflectivity=int(reflectivity);p.tag=int(tag);p.line=int(line)
        out.points.append(p)
    out.point_num=len(out.points)
    return out
def main():
    rospy.init_node('twin_livox_adapter')
    pub=rospy.Publisher('/livox/lidar',CustomMsg,queue_size=5)
    def cb(msg):
        try:pub.publish(convert(msg))
        except ValueError as e:rospy.logerr_throttle(5,str(e))
    rospy.Subscriber('/twin/lidar/points_timed',PointCloud2,cb,queue_size=5)
    rospy.spin()
if __name__=='__main__':main()
