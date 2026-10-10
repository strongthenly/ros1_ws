#!/usr/bin/env python3
import sys,unittest,threading,socket
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/twin_bridge/scripts'))
import bridge,livox_adapter,rospy,msgpack
from std_msgs.msg import Header
from sensor_msgs.msg import PointField
from sensor_msgs import point_cloud2
class Interfaces(unittest.TestCase):
    def test_frd_to_flu_gravity(self):
        self.assertEqual(bridge.flu(dict(x_val=0,y_val=0,z_val=-9.81)),(0,0,9.81))
    def test_plain_xyz_rejected(self):
        m=point_cloud2.create_cloud_xyz32(Header(stamp=rospy.Time(1),frame_id='lidar'),[(1,2,3)])
        with self.assertRaises(ValueError):livox_adapter.convert(m)
    def test_timing_sorted_preserved(self):
        fields=[PointField('x',0,PointField.FLOAT32,1),PointField('y',4,PointField.FLOAT32,1),PointField('z',8,PointField.FLOAT32,1),PointField('offset_time',12,PointField.UINT32,1),PointField('reflectivity',16,PointField.UINT8,1),PointField('tag',17,PointField.UINT8,1),PointField('line',18,PointField.UINT8,1)]
        m=point_cloud2.create_cloud(Header(stamp=rospy.Time(2,123),frame_id='lidar'),fields,[(1,2,3,100,4,16,0),(2,3,4,0,5,16,1)])
        out=livox_adapter.convert(m)
        self.assertEqual(out.timebase,2000000123);self.assertEqual([p.offset_time for p in out.points],[0,100]);self.assertEqual(out.point_num,2)
    def test_rpc_fragmented_response(self):
        server=socket.socket();server.bind(('127.0.0.1',0));server.listen(1)
        def serve():
            conn,_=server.accept();up=msgpack.Unpacker(raw=False)
            while True:
                up.feed(conn.recv(4096));requests=list(up)
                if requests:break
            req=requests[0];data=msgpack.packb([1,req[1],None,{'time_stamp':123,'ok':True}],use_bin_type=True)
            for b in data:conn.sendall(bytes([b]))
            conn.close();server.close()
        t=threading.Thread(target=serve);t.start()
        rpc=bridge.Rpc('127.0.0.1',server.getsockname()[1])
        self.assertEqual(rpc.call('ping',[]),{'time_stamp':123,'ok':True});rpc.sock.close();t.join()
if __name__=='__main__':unittest.main()
