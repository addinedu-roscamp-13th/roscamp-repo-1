#!/bin/bash
# =====================================================================
#  카메라↔베이스 핸드아이 캘리브 — 노트북 원클릭 (도메인20, J5/joint6 기준)
#  네 터미널에서:  bash ~/handeye_ws/캘리브_시작.sh
#
#  ★ 카메라는 pyrealsense2 SDK 직접 읽음 (골칫거리 realsense ROS노드 안 씀)
#  로봇쪽(dg_control_node + robot_state_publisher)은 이미 로봇에서 실행중
# =====================================================================
source /opt/ros/jazzy/setup.bash
source "$HOME/handeye_ws/install/setup.bash"
export ROS_DOMAIN_ID=20
D="$HOME/Desktop/tomato_pkg_extract/deploy"
trap 'echo; echo "[정리] 종료..."; pkill -f charuco_tf_direct 2>/dev/null; kill $(jobs -p) 2>/dev/null; exit 0' INT TERM

# 잔여 카메라 프로세스 정리(충돌 방지)
pkill -f realsense2_camera 2>/dev/null; pkill -f charuco_tf 2>/dev/null; sleep 1

echo "[1/2] charuco_tf_direct — 카메라 SDK직접 + 보드검출 → handeye_target TF"
python3 -u "$D/charuco_tf_direct.py" > /tmp/ctd.log 2>&1 &
sleep 5
echo "      (검출 로그: tail -f /tmp/ctd.log)"
grep -E "시작|검출 코너" /tmp/ctd.log 2>/dev/null | tail -2

echo "[2/2] easy_handeye2 캘리브 — rqt 창이 뜹니다 (Take Sample / Compute)"
echo "      base=g_base  effector=joint6  sensor=camera_color_optical_frame  target=handeye_target"
ros2 launch easy_handeye2 calibrate.launch.py \
  name:=jetcobot_handeye \
  calibration_type:=eye_in_hand \
  robot_base_frame:=g_base \
  robot_effector_frame:=joint6 \
  tracking_base_frame:=camera_color_optical_frame \
  tracking_marker_frame:=handeye_target
