# 시나리오2(수확) 아키텍처

상세: https://robot8.atlassian.net/wiki/x/GoBuAg (Confluence "시나리오2 수확 구현")

## 문제

MoveIt2 + Octomap으로 D435 pointcloud 기반 장애물 회피를 하는데, Octomap은
의미(semantic) 구분이 없어 필터링 없이는 수확 대상 토마토 자체도 장애물로
인식되어 회피 대상이 된다.

## 물리 구성 — 노트북 / RPi-5 역할 분리

연산(비전, 경로계획, Octomap)은 전부 **노트북**에서 처리하고, **RPi-5는
실물 서보 제어 미러링 전용**(연산 없음)이라는 게 핵심.

- **노트북**: D435(USB), `dg_ai_service`, `ddagi_control`, pointcloud 필터링
  노드(신규), `move_group`(MoveIt2 planning), `occupancy_map_monitor`(Octomap),
  `ros2_control_node`, `coord_to_goal_node`, `pymoveit2`, `easy_handeye2`.
- **RPi-5(jetcobot)**: `sync_plan`(`mycobot_280_moveit2_control`) —
  `/joint_states` 구독 → 각도 변환 → `pymycobot.send_angles()`로 실물 서보
  전달만 함. `follow_display`(`mycobot_280jn`)는 핸드-아이 캘리브레이션
  전용. **moveit2 관련 무거운 패키지를 RPi-5로 옮기지 않는다.**
- 노트북 ↔ RPi-5는 같은 `ROS_DOMAIN_ID`, 같은 LAN(WiFi).

## AI Service(`dg_ai_service`) 확장 — 토마토 인스턴스 마스킹

기존엔 프레임당 토마토 `id`/`grade`/좌표(x,y,z)만 반환(YOLO bbox 기반).
추가 예정: 개별 토마토 **인스턴스 세그멘테이션 마스크** + **겹치는 마스크
병합**, 기존 응답에 함께 포함. 마스크 생성은 AI Service가 이미 들고 있는
같은 프레임/모델에서 나오므로 이 서비스에 둔다 (automato_ws에 별도
추론 파이프라인을 또 만들지 않음).

`mycobot` 프로젝트의 `yolo_d435_detector_node`는 폐기 — 원래 mycobot 단독
검증용 임시 노드였고, 실제로는 `dg_ai_service`가 이 역할을 대체한다
(automato_ws로 이관한 `mycobot_280_pick`에도 포함 안 함).

## Pointcloud 필터링 (신규, 노트북)

D435 depth/pointcloud + `dg_ai_service`의 병합된 토마토 마스크 → 카메라
intrinsics/TF로 마스크를 depth에 정렬 → 마스크 영역 포인트 제거 →
필터링된 pointcloud를 `occupancy_map_monitor`에 공급. (mycobot 문서의
"접근 직전 국소 clear" 방식은 폐기하고 이 사전 마스킹 방식으로 대체 —
여러 토마토가 겹쳐 있어도 한 번에 처리되고, 경로 계획 시작 시점부터
토마토를 장애물로 보지 않는다.)

## 프로젝트 배치

| 프로젝트/패키지 | 위치 |
|---|---|
| `Eval_Yolo` | 노트북(개발용). 결과물 `.pt`만 전달 |
| `automato_ws`(`dg_ai_service`, `ddagi_control`, 필터 노드) | 노트북 |
| `automato_ws`(`mycobot_280_pick` 등 MoveIt2 관련, `pymoveit2`, `easy_handeye2`) | 노트북 |
| 서드파티(`mycobot_ros2`, `moveit_calibration` 업스트림 등) | 벤더링 안 함, `equip/automato_ws/mycobot.repos`(vcstool)로 clone |
| RPi-5 상주 코드(`sync_plan`, `follow_display`) | RPi-5 |

## YOLO 모델(.pt) 관리

`Eval_Yolo`에서 학습, 결과물 `.pt`만 `equip/automato_ws/src/dg_ai_service/models/`에
배치. `.gitignore`에서 `*.pt`/`*.onnx`는 전역 제외 — 파일은 git에 올리지 않고 별도 채널로 공유.
