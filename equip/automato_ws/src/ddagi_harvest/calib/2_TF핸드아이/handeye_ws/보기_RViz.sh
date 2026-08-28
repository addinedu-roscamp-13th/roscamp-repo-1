#!/bin/bash
# =====================================================================
#  RViz 보기 — 카메라 영상 + TF + ChArUco 검출 + (원하면 moveit 패널)
#  네 터미널에서 실행:  bash ~/handeye_ws/보기_RViz.sh
# =====================================================================
source /opt/ros/jazzy/setup.bash
source "$HOME/handeye_ws/install/setup.bash"
export ROS_DOMAIN_ID=20
D="$HOME/Desktop/tomato_pkg_extract/deploy"

echo "[확인] 카메라 토픽: $(ros2 topic list 2>/dev/null | grep -c camera)개"

# 카메라 안 떠있으면 기동
if ! pgrep -f realsense2_camera_node >/dev/null; then
  echo "[기동] realsense 카메라..."
  nohup ros2 launch realsense2_camera rs_launch.py enable_gyro:=false enable_accel:=false >/tmp/he_camera.log 2>&1 &
  sleep 12
fi

# ChArUco 검출→TF
if ! pgrep -f charuco_tf_publisher >/dev/null; then
  echo "[기동] charuco_tf_publisher (보드검출→handeye_target)..."
  nohup python3 -u "$D/charuco_tf_publisher.py" >/tmp/he_charuco.log 2>&1 &
  sleep 2
fi

# 로봇자세→TF (SSH via AUTOMATO 192.168.100.12 — 다중인터페이스라 DDS 불안정, SSH 사용)
if ! pgrep -f arm_tf_bridge >/dev/null; then
  echo "[기동] arm_tf_bridge (base_link->gripper_link, SSH 192.168.100.12)..."
  nohup python3 -u "$D/arm_tf_bridge.py" --euler ZYX >/tmp/he_armtf.log 2>&1 &
  sleep 3
fi

echo "[실행] RViz — 창이 뜹니다. (카메라영상 + TF축들)"
echo "  ▶ moveit ChArUco 패널 열기: 상단 Panels → Add New Panel → HandEye Calibration"
rviz2 -d "$HOME/handeye_ws/handeye_view.rviz"
