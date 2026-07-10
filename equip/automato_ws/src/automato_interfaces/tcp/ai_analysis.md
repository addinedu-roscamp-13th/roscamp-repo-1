# RP-50· DG Control Service ↔ DG AI Service (TCP)

> 출처: Confluence "Sprint 3 인터페이스 통합 테스트 메세지 규격" (#8, #9). 팀 합의 원문 그대로.
> 전송은 ROS2가 아닌 **TCP / JSON** 이라 rosidl 인터페이스가 아니며, 계약 문서로만 관리한다.

## 프레이밍 (메시지 경계)

TCP 는 스트림이라 메시지 경계가 없으므로 **길이 프리픽스 프레이밍**을 쓴다.

```
[ 4 bytes ] payload_size   # 뒤따르는 JSON 바이트 수. uint32, Big-Endian (network byte order)
[ N bytes ] payload        # UTF-8 로 인코딩한 JSON (N == payload_size)
```

- 송신: `payload = json.dumps(obj).encode('utf-8')` → `struct.pack('>I', len(payload)) + payload` 전송
- 수신: 먼저 4바이트를 읽어 `payload_size = struct.unpack('>I', header)[0]` → 이어서 정확히 그 크기만큼 읽어 UTF-8 JSON 파싱
- `>I` = Big-Endian unsigned int (4B). 부분 수신(short read)에 대비해 4B·N B 모두 **정확한 길이까지 반복 recv** 한다.

## #8  DG (DdaGoDdagi) Control Service → DG (DdaGoDdagi) AI Service
- 통신: TCP
- 내용: 건별 정밀 분석 요청 (집기 직전 1회) — 이 토마토 분류·grasp coord
- 흐름: 시작 → 분석 결과 → 종료

```
DG control -> AI
{
  "op" : "start | stop"
}
AI -> DG control
{
  "status" : "ripe" | "unripe" | "overripe" | "damaged",
  "coord" : [x, y, z],
  "confidence": 0.9,
}
```