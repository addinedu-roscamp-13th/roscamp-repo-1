#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
arm_tf_bridge — 로봇 자세(WiFi)를 로컬 ROS TF로 브리지
======================================================
WiFi 너머로는 /automato 토픽이 안 보임(DDS 차단). 그래서 paramiko(SSH)로
로봇의 /automato/arm_coords 를 읽어서, 노트북 로컬 ROS2에 TF 로 다시 발행한다.
  base_link → gripper_link  (핸드아이 캘리브가 이 TF를 읽음)

rx,ry,rz 오일러 규약은 pymycobot이 애매 → --euler 로 바꿔가며 검증(기본 ZYX).
실행: source ROS + install → python3 arm_tf_bridge.py [--euler ZYX]
"""
import argparse, json, math, threading, time
import paramiko
import rclpy
from rclpy.node import Node
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
        super().__init__('arm_tf_bridge')
        self.euler=euler
        self.br=TransformBroadcaster(self)
        self.coords=[0,0,300,0,0,0]; self.lock=threading.Lock()
        self.ssh=paramiko.SSHClient(); self.ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        self.ssh.connect("192.168.100.12",username="jetcobot",password="1",timeout=10)
        self.pre="source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=42; "
        threading.Thread(target=self.poll, daemon=True).start()
        self.create_timer(0.05, self.publish)  # 20Hz TF
        self.get_logger().info(f"arm_tf_bridge 시작 (euler={euler})")

    def poll(self):
        while True:
            try:
                o=self.ssh.exec_command(self.pre+"timeout 3 ros2 topic echo --once /automato/arm_coords 2>/dev/null")[1].read().decode()
                i=o.find("[")
                if i>=0:
                    c=json.loads("["+o[i+1:o.find("]")]+"]")
                    with self.lock: self.coords=c
            except Exception as e:
                self.get_logger().warn(f"poll: {e}"); time.sleep(1)
            time.sleep(0.15)

    def publish(self):
        with self.lock: c=list(self.coords)
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
    finally: n.ssh.close(); rclpy.shutdown()

if __name__=="__main__": main()
