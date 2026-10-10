#!/usr/bin/env python3
"""Synthetic protocol test server, not sensor simulation or algorithm validation."""
import socketserver,time,msgpack
start=time.monotonic_ns()
def now():return 1700000000000000000+time.monotonic_ns()-start
class Handler(socketserver.BaseRequestHandler):
    def handle(self):
        unpack=msgpack.Unpacker(raw=False)
        while True:
            data=self.request.recv(65536)
            if not data:return
            unpack.feed(data)
            for _,seq,method,args in unpack:
                ns=now();error=None
                if method=='getImuData':result={'time_stamp':ns,'angular_velocity':{'x_val':0.,'y_val':0.,'z_val':0.},'linear_acceleration':{'x_val':0.,'y_val':0.,'z_val':-9.81}}
                elif method=='getLidarData':result={'time_stamp':ns,'point_cloud':[1.,0.,0.,2.,1.,-1.]}
                elif method=='simGetImages':result=[dict(image_type=i,time_stamp=ns,width=16,height=12,image_data_float=[2.]*(16*12) if i else [],image_data_uint8=bytes([127])*(16*12*3) if not i else b'') for i in (0,1)]
                else:result=None;error='unsupported method'
                self.request.sendall(msgpack.packb([1,seq,error,result],use_bin_type=True))
class Server(socketserver.ThreadingTCPServer):allow_reuse_address=True;daemon_threads=True
if __name__=='__main__':
    with Server(('127.0.0.1',42459),Handler) as server:server.serve_forever()
