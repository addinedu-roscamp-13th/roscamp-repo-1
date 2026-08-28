#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
arm_tf_bridge_dds — 로봇 자세를 로컬 ROS TF로 브리지 (DDS 직통, SSH 불필요)
==========================================================================
노트북이 로봇과 같은 네트워크(jetcobot_aac0, 192.168.6.x)에 붙으면서
/automato/arm_coords 가 DDS로 직접 보인다. 그래서 SSH(paramiko) 없이
바로 구독해서 base_link → gripper_link TF 로 발행한다.
  (이전 arm_tf_bridge.py 는 WiFi DDS 차단 때문에 SSH로 우회했었음 — 이제 불필요)

/automato/arm_coords : std_msgs/String, data='{"coords":[x,y,z,rx,ry,rz], "ts":..}'
                       x,y,z=mm, rx,ry,rz=deg
rx,ry,rz 오일러 규약은 pymycobot이 애매 → --euler 로 바꿔가며 검증(기본 ZYX).
실행: source ROS + install → python3 arm_tf_bridge_dds.py [--euler ZYX]
"""
import argparse, json, math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

def euler_to_quat(rx, ry, rz, order):
    rx, ry, rz = map(math.radians, (rx, ry, rz))
    ca,sa=math.cos(rx/2),math.sin(rx/2); cb,sb=math.cos(ry/2),math.sin(ry/2); cc,sc=math.cos(rz/2),math.sin(rz/2)
    qx=(sa,0,0,ca); qy=(0,sb,0,cb); qz=(0,0,sc,cc)
    def mul(a,b):
        ax,ay,az,aw=a; bx,by,bz,bw=b
        return (aw*bx+ax*bw+ay*bz-az*by, aw*by-ax*bz+ay*bw+az*bx,
                aw*bz+ax*by-ay*bx+az*bw, aw*bw-ax*bx-ay*by-az*bz)
    q={'X':qx,'Y':qy,'Z':qz}; r=(0,0,0,1)
    for ax in order: r=mul(r,q[ax])
    return r

class Bridge(Node):
    def __init__(self, euler):
        super().__init__('arm_tf_bridge_dds')
        self.euler=euler
        self.br=TransformBroadcaster(self)
        self.coords=[0,0,300,0,0,0]; self.got=False
        self.create_subscription(String, '/automato/arm_coords', self.on_coords, 10)
        self.create_timer(0.05, self.publish)  # 20Hz TF
        self.get_logger().info(f"arm_tf_bridge_dds 시작 (euler={euler}) — /automato/arm_coords DDS 구독")

    def on_coords(self, msg):
        try:
            c=json.loads(msg.data).get("coords")
            if isinstance(c,list) and len(c)==6:
                self.coords=c
                if not self.got:
                    self.got=True; self.get_logger().info(f"  첫 arm_coords 수신: {[round(v,1) for v in c]}")
        except Exception as e:
            self.get_logger().warn(f"파싱: {e}")

    def publish(self):
        c=self.coords
        t=TransformStamped()
        t.header.stamp=self.get_clock().now().to_msg()
        t.header.frame_id='base_link'; t.child_frame_id='gripper_link'
        t.transform.translation.x=c[0]/1000.0; t.transform.translation.y=c[1]/1000.0; t.transform.translation.z=c[2]/1000.0
        qx,qy,qz,qw=euler_to_quat(c[3],c[4],c[5],self.euler)
        t.transform.rotation.x=qx; t.transform.rotation.y=qy; t.transform.rotation.z=qz; t.transform.rotation.w=qw
        self.br.sendTransform(t)

def main():
    ap=argparse.ArgumentParser(); ap.add_argument("--euler",default="ZYX"); a,_=ap.parse_known_args()
    rclpy.init(); n=Bridge(a.euler)
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    finally: rclpy.try_shutdown()

if __name__=="__main__": main()
