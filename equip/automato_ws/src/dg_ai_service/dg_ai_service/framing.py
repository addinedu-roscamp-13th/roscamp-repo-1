#!/usr/bin/env python3
"""RP-50 TCP 프레이밍 헬퍼.

메시지 경계: [ 4B payload_size (uint32, Big-Endian) ] + [ N B UTF-8 JSON ]
(dg_control.framing 과 동일 내용 — 공용 계약)
"""
import json
import socket
import struct


def send_msg(sock: socket.socket, obj: dict) -> None:
    """dict 를 JSON 으로 직렬화해 [4B 길이][JSON] 형식으로 전송."""
    payload = json.dumps(obj, ensure_ascii=False).encode('utf-8')
    sock.sendall(struct.pack('>I', len(payload)) + payload)


def _recv_exactly(sock: socket.socket, n: int) -> bytes:
    """정확히 n 바이트를 읽을 때까지 반복 recv (short read 대비)."""
    buf = b''
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError('상대가 연결을 닫음')
        buf += chunk
    return buf


def recv_msg(sock: socket.socket) -> dict:
    """[4B 길이][JSON] 한 개를 읽어 dict 로 반환."""
    size = struct.unpack('>I', _recv_exactly(sock, 4))[0]
    return json.loads(_recv_exactly(sock, size).decode('utf-8'))
