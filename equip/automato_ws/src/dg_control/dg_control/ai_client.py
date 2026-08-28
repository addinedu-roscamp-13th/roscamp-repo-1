#!/usr/bin/env python3
"""DG Control Service(DCS) → DG AI Service TCP 클라이언트.

시퀀스 다이어그램 E2-3/4 규격:
  - 연결: 지속(persistent) TCP. 매 요청마다 새로 열지 않음.
  - 프레이밍: [4B payload_size(big-endian uint32)][UTF-8 JSON]
  - Request : analyze_frame_request  (task_id, waypoint_id, image_encoding, image_data<base64>)
  - Response: analyze_frame_response (status=OK -> result{ripe/unripe/rotten/disease_percent})

접속 대상(엔드포인트)은 dg_web/dg_ai_target.json 의 active("real"|"sim") 값을 따른다.
  - dg_ai 시뮬을 끄면 대시보드가 active 를 "real" 로 바꾼다 → 다음 요청에서 실서버로 자동 전환.
  - 파일이 없거나 못 읽으면 생성자 인자 default_endpoint 로 폴백.
"""
import json
import os
import socket
import struct
import threading


def _abbrev(obj, maxlen=140):
    """로그용: 긴 문자열(예: base64 이미지)은 앞부분+길이로 요약. dict/list 구조는 유지."""
    if isinstance(obj, dict):
        return {k: _abbrev(v, maxlen) for k, v in obj.items()}
    if isinstance(obj, list):
        out = [_abbrev(v, maxlen) for v in obj[:20]]
        if len(obj) > 20:
            out.append('...(%d items)' % len(obj))
        return out
    if isinstance(obj, str) and len(obj) > maxlen:
        return '%s...(%d chars)' % (obj[:maxlen], len(obj))
    return obj


def _parse_hostport(s, default_port=9100):
    """'127.0.0.1:9100' -> ('127.0.0.1', 9100). 포트 생략 시 default_port."""
    s = (s or '').strip()
    if not s:
        return None
    if ':' in s:
        host, port = s.rsplit(':', 1)
        return host, int(port)
    return s, default_port


class AiTcpClient:
    """AI Service 와의 지속 TCP 연결 + 엔드포인트 자동 전환 관리."""

    def __init__(self, target_file, default_endpoint='127.0.0.1:9100',
                 connect_timeout=3.0, io_timeout=5.0, logger=None):
        self.target_file = target_file
        self.default_endpoint = default_endpoint
        self.connect_timeout = connect_timeout
        self.io_timeout = io_timeout
        self.log = logger
        self._sock = None
        self._cur_endpoint = None          # 현재 소켓이 붙어있는 'host:port'
        self._lock = threading.Lock()

    # ----- 엔드포인트 결정 (dg_ai_target.json 의 active) -----
    def resolve_target(self):
        """(endpoint, active) 반환. active 는 'real'|'sim', 파일을 못 읽거나 값이
        비면 'default'(= default_endpoint 폴백)."""
        try:
            with open(self.target_file, 'r', encoding='utf-8') as f:
                cfg = json.load(f)
            active = cfg.get('active', 'sim')
            ep = cfg.get(active)
            if ep:
                return ep.strip(), active
        except (OSError, ValueError, KeyError):
            pass
        return self.default_endpoint, 'default'

    def resolve_endpoint(self):
        return self.resolve_target()[0]

    def probe(self):
        """지금 접속 대상에 실제로 붙어본다(기동 시 연결 확인용).

        analyze() 와 같은 지속 소켓을 쓰므로, 성공하면 그 연결을 그대로 두고
        첫 분석 요청이 재사용한다(이미 붙어 있으면 아무 것도 하지 않음).
        반환: {'endpoint', 'active', 'ok', 'error'}
        """
        ep, active = self.resolve_target()
        with self._lock:
            try:
                self._ensure_connected_locked()
                return {'endpoint': self._cur_endpoint or ep, 'active': active,
                        'ok': True, 'error': ''}
            except (OSError, ConnectionError) as e:
                self._close_locked()
                return {'endpoint': ep, 'active': active,
                        'ok': False, 'error': str(e)}

    def _log(self, msg):
        if self.log is not None:
            self.log.info(msg)

    def _close_locked(self):
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
        self._sock = None
        self._cur_endpoint = None

    def _ensure_connected_locked(self):
        """원하는 엔드포인트로 접속돼 있는지 확인. 바뀌었으면 재접속(자동 전환)."""
        want = self.resolve_endpoint()
        if self._sock is not None and self._cur_endpoint == want:
            return
        # 엔드포인트가 바뀌었거나 소켓이 없으면 새로 연결
        self._close_locked()
        hp = _parse_hostport(want)
        if hp is None:
            raise ConnectionError('AI 엔드포인트 미설정')
        host, port = hp
        s = socket.create_connection((host, port), timeout=self.connect_timeout)
        s.settimeout(self.io_timeout)
        self._sock = s
        self._cur_endpoint = want
        self._log('AI Service 접속: %s' % want)

    @staticmethod
    def _send_frame(sock, obj):
        payload = json.dumps(obj).encode('utf-8')
        sock.sendall(struct.pack('>I', len(payload)) + payload)

    @staticmethod
    def _recv_exact(sock, n):
        buf = b''
        while len(buf) < n:
            chunk = sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError('AI Service 연결 종료됨')
            buf += chunk
        return buf

    def _recv_frame(self, sock):
        header = self._recv_exact(sock, 4)
        size = struct.unpack('>I', header)[0]
        return json.loads(self._recv_exact(sock, size).decode('utf-8'))

    def analyze(self, request_id, task_id, waypoint_id,
                image_encoding='jpeg', image_data=''):
        """분석 요청 1회. 성공 시 **응답 dict 전체**(result·labeled_image 등 포함), 실패 시 예외.
        한 번 실패하면 소켓을 버리고 1회 재시도(엔드포인트 재해석 포함)."""
        req = {
            'message_type': 'analyze_frame_request',
            'request_id': request_id,
            'task_id': int(task_id),
            'waypoint_id': int(waypoint_id),
            'image_encoding': image_encoding,
            'image_data': image_data,
        }
        with self._lock:
            last_err = None
            for _attempt in range(2):
                try:
                    self._ensure_connected_locked()
                    self._send_frame(self._sock, req)
                    resp = self._recv_frame(self._sock)
                    # AI 응답 원본 JSON을 로그로 남긴다(큰 base64 필드는 길이만 요약).
                    # 병충해 검출 시 이미지가 포함될 수 있어 실제 형태 파악용.
                    self._log('AI 응답 원본 JSON(wp=%s): %s'
                              % (waypoint_id, json.dumps(_abbrev(resp), ensure_ascii=False)))
                    if resp.get('status') == 'OK':
                        return resp
                    raise RuntimeError('AI 분석 오류: %s' % resp.get('error_code'))
                except (OSError, ConnectionError) as e:
                    last_err = e
                    self._close_locked()   # 다음 시도에서 재접속(자동 전환 기회)
            raise ConnectionError('AI Service 요청 실패: %s' % last_err)

    def close(self):
        with self._lock:
            self._close_locked()
