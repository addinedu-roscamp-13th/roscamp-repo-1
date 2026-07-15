#!/usr/bin/env python3
"""DG AI Service 시뮬레이터 (TCP 서버).

팀원이 개발 중인 실제 DG AI Service 대역. DCS 가 붙는 지속 TCP 연결을 받아
analyze_frame_request 를 받으면 즉시 가짜 분석 결과(analyze_frame_response)를 돌려준다.
DCS 가 붙는 쪽이다(실서버 대역).

프레이밍(시퀀스 다이어그램 E2-3/4): [4B payload_size(big-endian)][UTF-8 JSON]

가짜 결과 규칙(테스트 재현성): ripe/unripe/rotten/disease percent, 합계 100.
  disease = 5 if wp%3==0 else 0 (E3 트리거 disease>=5 확인용), 나머지로 ripe 채움.

받은 image_data(base64 JPEG)는 디코드해 파일로 저장한다(수신 확인용).

환경변수:
  DG_AI_SIM_HOST (기본 0.0.0.0)
  DG_AI_SIM_PORT (기본 9100)
  DG_AI_SIM_SAVE_DIR (기본 automato_ws/dg_ai_recv) — 수신 이미지 저장 폴더

실행: ros2 run dg_sim dg_ai_sim   (ROS 의존 없음, 순수 TCP)
"""
import base64
import io
import json
import os
import socket
import struct
import threading
import time

# 분석 처리 시간 시뮬(응답 지연). 환경변수 DG_AI_SIM_DELAY(기본 3.0초). main()에서 설정.
RESP_DELAY = 3.0
# E3 병해충 알림 발동 기준(이 값 이상일 때만 라벨 이미지를 실어 보낸다)
DISEASE_ALERT_PCT = 5
# 수신 이미지 저장 폴더
SAVE_DIR = os.environ.get(
    'DG_AI_SIM_SAVE_DIR',
    '/home/ane/dev_ws/roscamp-rp108-navigate/equip/automato_ws/dg_ai_recv')


def save_received_image(req):
    """analyze_frame_request 의 image_data(base64 JPEG)를 디코드해 파일로 저장."""
    b64 = req.get('image_data', '')
    if not b64 or b64.startswith('<'):   # wire 플레이스홀더 방어
        return
    try:
        raw = base64.b64decode(b64)
    except (ValueError, TypeError):
        return
    try:
        os.makedirs(SAVE_DIR, exist_ok=True)
    except OSError:
        return
    base = 'recv_task%s_wp%s' % (req.get('task_id'), req.get('waypoint_id'))
    try:
        from PIL import Image as PILImage
        im = PILImage.open(io.BytesIO(raw))   # JPEG 디코드
        path = os.path.join(SAVE_DIR, base + '.jpg')
        im.save(path)
        print('[dg_ai_sim] 수신 이미지 저장: %s (%dx%d, %d bytes)'
              % (path, im.width, im.height, len(raw)), flush=True)
    except Exception as e:   # noqa: BLE001 — 디코드 실패 시 raw 저장
        path = os.path.join(SAVE_DIR, base + '.bin')
        try:
            with open(path, 'wb') as f:
                f.write(raw)
            print('[dg_ai_sim] 이미지 디코드 실패 → raw 저장: %s (%d bytes): %s'
                  % (path, len(raw), e), flush=True)
        except OSError:
            pass


def make_result(waypoint_id):
    # 익음/덜익음/부패/병해 percent (합계 100). 일부 waypoint에서 disease>0(E3 확인용).
    disease = 5 if waypoint_id % 3 == 0 else 0
    rotten = 10 if waypoint_id % 2 == 1 else 5
    unripe = 20 + (waypoint_id % 4) * 5
    ripe = 100 - unripe - rotten - disease
    return {
        'ripe_percent': ripe,
        'unripe_percent': unripe,
        'rotten_percent': rotten,
        'disease_percent': disease,
    }


def _recv_exact(conn, n):
    buf = b''
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return buf


def _send_frame(conn, obj):
    payload = json.dumps(obj).encode('utf-8')
    conn.sendall(struct.pack('>I', len(payload)) + payload)


def handle_conn(conn, addr):
    print('[dg_ai_sim] 연결: %s' % (addr,), flush=True)
    try:
        while True:
            header = _recv_exact(conn, 4)
            if header is None:
                break
            size = struct.unpack('>I', header)[0]
            body = _recv_exact(conn, size)
            if body is None:
                break
            try:
                req = json.loads(body.decode('utf-8'))
            except ValueError:
                continue
            if req.get('message_type') != 'analyze_frame_request':
                _send_frame(conn, {'message_type': 'analyze_frame_response',
                                   'request_id': req.get('request_id'),
                                   'status': 'ERROR', 'error_code': 'BAD_REQUEST',
                                   'error_message': 'unknown message_type'})
                continue
            wp = int(req.get('waypoint_id', 0))
            save_received_image(req)     # 수신 이미지 디코드·저장
            if RESP_DELAY > 0:
                time.sleep(RESP_DELAY)   # 분석 처리 시간 시뮬
            resp = {
                'message_type': 'analyze_frame_response',
                'request_id': req.get('request_id'),
                'status': 'OK',
                'result': make_result(wp),
            }
            # 병해충 disease_percent >= 5 일 때만 라벨링 이미지 동봉(E3 규격, 실서버 형태 모사:
            # labeled_image + labeled_image_encoding). 실제 어노테이션 대신 입력 이미지를 에코.
            if resp['result'].get('disease_percent', 0) >= DISEASE_ALERT_PCT and req.get('image_data'):
                resp['result']['labeled_image'] = req['image_data']
                resp['result']['labeled_image_encoding'] = 'jpeg'
            _send_frame(conn, resp)
            has_img = 'labeled_image' in resp['result']
            print('[dg_ai_sim] wp=%d disease=%d labeled_image=%s'
                  % (wp, resp['result']['disease_percent'], has_img), flush=True)
    except OSError:
        pass
    finally:
        conn.close()
        print('[dg_ai_sim] 연결 종료: %s' % (addr,), flush=True)


def main(args=None):
    global RESP_DELAY
    RESP_DELAY = float(os.environ.get('DG_AI_SIM_DELAY', '3.0'))
    host = os.environ.get('DG_AI_SIM_HOST', '0.0.0.0')
    port = int(os.environ.get('DG_AI_SIM_PORT', '9100'))
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port))
    srv.listen(5)
    print('[dg_ai_sim] TCP 대기 %s:%d' % (host, port), flush=True)
    try:
        while True:
            conn, addr = srv.accept()
            threading.Thread(target=handle_conn, args=(conn, addr), daemon=True).start()
    except KeyboardInterrupt:
        pass
    finally:
        srv.close()


if __name__ == '__main__':
    main()
