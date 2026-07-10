# services 개발 환경 설정

SERVICES 계층(Web/Control/DB) 실행·테스트 의존성. **새로 설치하면 여기에 추가.**

## 기본 환경
| 항목 | 값 |
|---|---|
| OS | Ubuntu (Linux) |
| ROS2 | Jazzy (`/opt/ros/jazzy`) — Control Service의 OpHarvest 액션용 |
| Python | 3.12 |

## 설치
```bash
# Web Service (순수 Flask)
pip install flask

# Control Service (ROS2 액션 클라이언트) — automato_interfaces 필요
#  equip/automato_ws 에서 automato_interfaces 빌드 후 source
cd ../equip/automato_ws
colcon build --packages-select automato_interfaces   # 빌드 의존: empy, catkin_pkg, lark, numpy, pyyaml
source install/setup.bash
```
> automato_interfaces 빌드가 `No module named 'em'/'catkin_pkg'` 등을 내면:
> `pip install "empy==3.3.4" catkin_pkg lark numpy pyyaml` (equip/SETUP.md 참고)

## 변경 이력
| 날짜 | 내용 |
|---|---|
| 2026-07-02 | 최초 작성. Web(flask) + Control(rclpy+automato_interfaces) 세팅 |
