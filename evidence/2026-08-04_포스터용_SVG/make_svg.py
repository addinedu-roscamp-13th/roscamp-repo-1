#!/usr/bin/env python3
"""대시보드를 **벡터 SVG** 로 뽑는다 (포스터용).

왜 스크린샷이 아니라 SVG 인가:
  PNG 로 캡처해 포스터에 크게 넣으면 글자가 깨진다(이건수 2026-08-05).
  Chrome 의 printToPDF 는 글자·도형을 벡터로 남기므로, 이를 pdftocairo 로 SVG 변환하면
  아무리 확대해도 글자가 선명하다. (히트맵·3D 는 <canvas> 라 그 부분만 래스터로 들어간다)

왜 `--print-to-pdf` 플래그 대신 CDP 인가:
  플래그로 뽑으면 용지가 Letter(612pt)로 고정돼 레이아웃이 그 폭에 눌린다 —
  데스크톱 2단 구성이 1단으로 접혔다. CDP 의 Page.printToPDF 는 paperWidth 를
  지정할 수 있어 화면과 같은 폭으로 레이아웃을 잡을 수 있다.

테마·뷰는 앱이 URL 쿼리를 지원한다: ?theme=light|dark & view=desktop|mobile
"""
import base64
import json
import os
import subprocess
import sys
import time

import websocket  # noqa: E402  (시스템에 설치돼 있음)

BASE = "http://127.0.0.1:7000"
OUT = os.path.dirname(os.path.abspath(__file__))
PORT = 9333


def cdp(ws, method, params=None, _id=[0]):
    _id[0] += 1
    ws.send(json.dumps({"id": _id[0], "method": method, "params": params or {}}))
    while True:
        msg = json.loads(ws.recv())
        if msg.get("id") == _id[0]:
            if "error" in msg:
                raise RuntimeError("%s: %s" % (method, msg["error"]))
            return msg.get("result", {})


def launch():
    p = subprocess.Popen(
        # --disable-gpu 를 주면 WebGL 컨텍스트를 못 만들어 3D 지도가
        # "Error creating WebGL context" 로 비어 나온다. SwiftShader(소프트웨어 GL)로 그린다.
        ["google-chrome", "--headless=new", "--no-sandbox",
         "--use-gl=angle", "--use-angle=swiftshader", "--enable-unsafe-swiftshader",
         "--hide-scrollbars", "--remote-debugging-port=%d" % PORT,
         # CDP 웹소켓은 기본적으로 교차 출처를 막는다(403). 로컬 전용이라 허용한다.
         "--remote-allow-origins=*",
         "--user-data-dir=/tmp/chrome-svg-profile", "about:blank"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    import urllib.request
    for _ in range(40):
        time.sleep(0.5)
        try:
            urllib.request.urlopen("http://127.0.0.1:%d/json/version" % PORT, timeout=2)
            return p
        except Exception:
            continue
    raise RuntimeError("Chrome 기동 실패")


def new_tab():
    """탭 하나를 얻는다. 최신 Chrome 은 /json/new 가 PUT 이라 GET 이면 405 가 난다.
       매번 새 탭을 열 필요도 없으므로 기동 시 만들어진 about:blank 탭을 재사용한다."""
    import urllib.request
    r = urllib.request.urlopen("http://127.0.0.1:%d/json/list" % PORT, timeout=5)
    for t in json.load(r):
        if t.get("type") == "page" and t.get("webSocketDebuggerUrl"):
            return t["webSocketDebuggerUrl"]
    # 없으면 PUT 으로 새로 만든다
    req = urllib.request.Request("http://127.0.0.1:%d/json/new" % PORT, method="PUT")
    return json.load(urllib.request.urlopen(req, timeout=5))["webSocketDebuggerUrl"]


def render(name, url, w, h, page_range, wait=14):   # 소프트웨어 GL 은 느리다
    ws = websocket.create_connection(new_tab(), timeout=90)
    try:
        cdp(ws, "Page.enable")
        # 화면과 같은 폭으로 레이아웃을 잡는다. 이걸 안 하면 용지 폭에 눌린다.
        cdp(ws, "Emulation.setDeviceMetricsOverride",
            {"width": w, "height": h, "deviceScaleFactor": 1, "mobile": False})
        cdp(ws, "Page.navigate", {"url": url})
        time.sleep(wait)          # 히트맵 canvas·3D 가 그려질 시간
        pdf = os.path.join(OUT, name + ".pdf")
        res = cdp(ws, "Page.printToPDF", {
            "printBackground": True, "preferCSSPageSize": False,
            "marginTop": 0, "marginBottom": 0, "marginLeft": 0, "marginRight": 0,
            "paperWidth": w / 96.0, "paperHeight": h / 96.0,
            "pageRanges": page_range,
        })
        with open(pdf, "wb") as f:
            f.write(base64.b64decode(res["data"]))
        return pdf
    finally:
        ws.close()


def to_svg(pdf, page):
    svg = pdf[:-4] + ".svg"
    subprocess.run(["pdftocairo", "-svg", "-f", str(page), "-l", str(page), pdf, svg],
                   check=True, capture_output=True)
    return svg


def pages(pdf):
    out = subprocess.run(["pdfinfo", pdf], capture_output=True, text=True).stdout
    for line in out.splitlines():
        if line.startswith("Pages:"):
            return int(line.split()[1])
    return 1


def last_content_page(pdf, n):
    """내용이 있는 **마지막** 쪽 번호. 그냥 마지막 쪽을 쓰면 안 된다 —
       페이지 나눔 때문에 끝쪽이 배경만 남은 빈 여백인 경우가 있다
       (실제로 모바일 다크가 3쪽이었는데 3쪽은 거의 빈 화면이었다)."""
    from PIL import Image
    import glob
    import tempfile
    best = 1
    with tempfile.TemporaryDirectory() as td:
        pre = os.path.join(td, "p")
        subprocess.run(["pdftocairo", "-png", "-r", "18", pdf, pre],
                       check=True, capture_output=True)
        for f in sorted(glob.glob(pre + "*.png")):
            page = int(os.path.basename(f).rsplit("-", 1)[1].split(".")[0])
            im = Image.open(f).convert("RGB")
            px = list(im.getdata())
            bg = max(set(px), key=px.count)          # 가장 많은 색 = 배경
            ink = sum(1 for c in px if sum(abs(a - b) for a, b in zip(c, bg)) > 30)
            if ink / len(px) > 0.06:                  # 6% 넘게 뭔가 그려져 있으면 '내용 있음'
                best = page
    return best


if __name__ == "__main__":
    proc = launch()
    try:
        jobs = [
            # 이름                      URL 쿼리                       폭    높이  쓸 페이지
            ("01_데스크톱_라이트", "?theme=light&view=desktop", 1600, 1150, "1"),
            ("02_모바일_상단_라이트", "?theme=light&view=mobile", 430, 920, "1"),
            ("03_모바일_하단_다크", "?theme=dark&view=mobile", 430, 920, "last"),
        ]
        for name, q, w, h, want in jobs:
            rng = "1" if want != "last" else ""
            pdf = render(name, BASE + "/" + q, w, h, rng)
            n = pages(pdf)
            page = last_content_page(pdf, n) if want == "last" else 1
            svg = to_svg(pdf, page)
            kb = os.path.getsize(svg) / 1024
            print("  %-24s %d쪽 중 %d쪽 → %s (%.0f KB)"
                  % (name, n, page, os.path.basename(svg), kb))
    finally:
        proc.terminate()
