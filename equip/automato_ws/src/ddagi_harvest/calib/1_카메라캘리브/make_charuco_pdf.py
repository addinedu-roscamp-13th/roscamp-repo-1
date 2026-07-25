#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
make_charuco_pdf — 인쇄용 ChArUco 보드 PDF 생성 (크기 정확)
==========================================================
물리보드와 동일 규격으로, 인쇄했을 때 square가 정확히 30mm가 되도록 A4에 배치.
  DICT_5X5_100, 5x7칸, square 30mm, marker 23mm  (스테레오 캘리브 성공 규격)
charuco_tf_publisher.py / calibrate_stereo_ir.py 의 검출 규격과 일치.

출력: /home/ane/Desktop/차르코_인쇄_2장.pdf  (2페이지 = 2장, 동일 보드)
⚠️ 인쇄 시 '실제 크기/100%' 로 (페이지 맞춤 끄기). 30mm 눈금자로 검증.
"""
import cv2, numpy as np
from reportlab.pdfgen import canvas
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.lib.utils import ImageReader
from PIL import Image

SX, SY = 5, 7
SQ_MM, MK_MM = 30.0, 23.0
DICT_ID = cv2.aruco.DICT_5X5_100
OUT = "/home/ane/Desktop/차르코_인쇄_2장.pdf"
N_COPIES = 2

# 보드 이미지 생성 (10 px/mm)
d = cv2.aruco.getPredefinedDictionary(DICT_ID)
board = cv2.aruco.CharucoBoard_create(SX, SY, SQ_MM/1000, MK_MM/1000, d)
w_px, h_px = int(SX*SQ_MM*10), int(SY*SQ_MM*10)   # 1500 x 2100
img = board.draw((w_px, h_px), marginSize=0, borderBits=1)
pil = Image.fromarray(img)

bw_mm, bh_mm = SX*SQ_MM, SY*SQ_MM                 # 150 x 210 mm
pw, ph = A4                                       # 595.27 x 841.89 pt
c = canvas.Canvas(OUT, pagesize=A4)
for i in range(N_COPIES):
    x = (pw - bw_mm*mm)/2
    y = (ph - bh_mm*mm)/2 + 8*mm
    c.drawImage(ImageReader(pil), x, y, width=bw_mm*mm, height=bh_mm*mm)
    # 규격 캡션
    c.setFont("Helvetica", 10)
    c.drawCentredString(pw/2, y-6*mm,
        f"ChArUco  DICT_5X5_100  {SX}x{SY}  square={SQ_MM:.0f}mm  marker={MK_MM:.0f}mm   (#{i+1}/{N_COPIES})")
    c.drawCentredString(pw/2, y-11*mm, "PRINT AT ACTUAL SIZE 100% (no fit-to-page). Verify 30mm scale below.")
    # 30mm 검증 눈금자
    rx, ry = (pw-30*mm)/2, y-18*mm
    c.setLineWidth(1); c.line(rx, ry, rx+30*mm, ry)
    c.line(rx, ry-1.5*mm, rx, ry+1.5*mm); c.line(rx+30*mm, ry-1.5*mm, rx+30*mm, ry+1.5*mm)
    c.setFont("Helvetica", 8); c.drawCentredString(pw/2, ry-4*mm, "|<-- 30 mm -->|")
    c.showPage()
c.save()
print(f"저장: {OUT}  ({N_COPIES}페이지, 보드 {bw_mm:.0f}x{bh_mm:.0f}mm)")
