#!/bin/bash
# =====================================================================
#  ② 카메라↔베이스 핸드아이 (easy_handeye2 + ChArUco) — 원클릭 런처
# =====================================================================
#  구성(모두 로컬, move_group/MoveIt 실물제어 없음 → JetCobot 안전):
#    [1] realsense 카메라 노드      : /camera/camera/color/image_raw ...
#    [2] charuco_tf_publisher.py    : camera_color_optical_frame -> handeye_target
#    [3] arm_tf_bridge.py           : base_link -> gripper_link   (SSH로 로봇자세 읽음)
#    [4] easy_handeye2 calibrate    : 위 TF들로 핸드아이 solve (rqt GUI)
#
#  종료: 이 터미널에서 Ctrl+C  (자식 프로세스 전부 정리됨)
# =====================================================================
set -u
source /opt/ros/jazzy/setup.bash
source "$HOME/handeye_ws/install/setup.bash"
export ROS_DOMAIN_ID=42
DEPLOY="$HOME/Desktop/tomato_pkg_extract/deploy"

PIDS=()
cleanup() {
  echo; echo "[정리] 자식 프로세스 종료..."
  for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null; done
  pkill -f charuco_tf_publisher 2>/dev/null
  pkill -f arm_tf_bridge 2>/dev/null
  # 카메라 노드는 남겨두려면 아래 줄 주석. 깨끗이 끄려면 유지:
  # pkill -f realsense2_camera_node 2>/dev/null
  echo "[정리] 완료"; exit 0
}
trap cleanup INT TERM

# ---- [1] 카메라 (이미 떠있으면 재사용) ----
if pgrep -f realsense2_camera_node >/dev/null; then
  echo "[1] 카메라 이미 실행중 — 재사용"
else
  echo "[1] realsense 카메라 기동 (12초 초기화)..."
  ros2 launch realsense2_camera rs_launch.py enable_gyro:=false enable_accel:=false \
    >/tmp/he_camera.log 2>&1 &
  PIDS+=($!); sleep 12
fi
NCAM=$(ros2 topic list 2>/dev/null | grep -c camera)
echo "    카메라 토픽 ${NCAM}개"

# ---- [2] ChArUco → TF ----
echo "[2] charuco_tf_publisher 기동 (camera->handeye_target)..."
python3 "$DEPLOY/charuco_tf_publisher.py" >/tmp/he_charuco.log 2>&1 &
PIDS+=($!); sleep 2

# ---- [3] 로봇자세 → TF ----
echo "[3] arm_tf_bridge 기동 (base_link->gripper_link, SSH)..."
python3 "$DEPLOY/arm_tf_bridge.py" --euler "${EULER:-ZYX}" >/tmp/he_armtf.log 2>&1 &
PIDS+=($!); sleep 3

# ---- [4] easy_handeye2 캘리브 GUI ----
echo "[4] easy_handeye2 캘리브 실행 — rqt GUI 창이 뜹니다."
echo "    (GUI에서: 로봇 자세 바꿔가며 Take Sample 15~20회 → Compute → Save)"
ros2 launch easy_handeye2 calibrate.launch.py \
  name:=jetcobot_handeye \
  calibration_type:=eye_in_hand \
  robot_base_frame:=base_link \
  robot_effector_frame:=gripper_link \
  tracking_base_frame:=camera_color_optical_frame \
  tracking_marker_frame:=handeye_target

cleanup
