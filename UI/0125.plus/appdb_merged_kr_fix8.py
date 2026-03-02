#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
appdb_merged_kr_fix8.py
======================

이번 수정은 사용자 요청사항을 반영합니다.

────────────────────────────────────────────────────────────
[fix8 수정 내역]
────────────────────────────────────────────────────────────
1. 웹캠 화면 크기 원상태 복구 (더 크게 보이도록)
2. 웹캠 영상에 4x4 격자(섹션) 표시 추가
3. AMR 화면 상단/하단 정보 완전 제거 (바운딩 박스만 표시)
4. DB 표는 좌우 분리 유지
5. 두 가지 HTML 버전 지원:
   - dashboard_v8a_basic.html: 기본 버전
   - dashboard_v8b_modal_new.html: 영상 클릭 시 확대 보기 기능 포함

────────────────────────────────────────────────────────────
[이전 버전에서 유지되는 기능]
────────────────────────────────────────────────────────────
- AMR1 이벤트에 붙는 section_id는 무조건 WEB1에서 가져온다.
- AMR2 이벤트에 붙는 section_id는 무조건 WEB2에서 가져온다.

즉, "가장 최근 웹캠" 같은 동적 선택을 하지 않습니다.
(사용자 요구: 매핑 규칙 고정)

────────────────────────────────────────────────────────────
[2] 웹 대시보드에서 4개 화면 + DB 표가 '한 화면(1페이지)'에 들어오게 한다
────────────────────────────────────────────────────────────
- 페이지 전체 높이를 100vh(브라우저 한 화면 높이)로 고정
- 위쪽에 2x2 영상, 아래쪽에 DB 요약표(AMR1/AMR2 최신 1개씩)를 배치
- DB 표는 높이를 제한하고, 내용이 많아지면 표 영역 내부에서만 스크롤

※ "작은 화면이면 1열로 내려간다" 같은 동작은 제거했습니다.
   항상 2열(좌/우)로 고정합니다.

────────────────────────────────────────────────────────────
[3] 클래스 저장 관련
────────────────────────────────────────────────────────────
- AMR 이벤트에는 "AMR이 감지한 class"만 저장합니다. (기존과 동일)
- 웹캠 쪽에서는 "섹션만" AMR에 붙이기 위한 목적으로 최신 섹션 상태만 유지합니다.
  (webcam_state 테이블을 유지하되, class_id는 의미 없으므로 0으로만 기록합니다)
- Web 관련 이벤트는 DB에 저장하지 않습니다.

────────────────────────────────────────────────────────────
[실행 방법]
────────────────────────────────────────────────────────────
cd /home/rokey/rokey_ws/src/to_students-main/day5/0122/
source /opt/ros/humble/setup.bash
source ~/rokey_ws/install/setup.bash
export ROS_DOMAIN_ID=5
export ROS_LOCALHOST_ONLY=0

# AMR 전용 YOLO pt 경로(반드시 실제 파일로)
export YOLO_AMR_MODEL_PATH=/home/rokey/rokey_ws/src/어딘가/amr_best.pt

# HTML 버전 선택 (선택사항)
# export HTML_VERSION=basic   # 기본 버전 (기본값)
# export HTML_VERSION=modal   # 확대 기능 포함 버전

python3 appdb_merged_kr_fix8.py

────────────────────────────────────────────────────────────
[HTML 파일 준비]
────────────────────────────────────────────────────────────
Python 파일과 같은 디렉토리에 HTML 파일을 배치하세요:
- dashboard_v8a_basic.html: 기본 버전 (격자 표시 포함)
- dashboard_v8b_modal_new.html: 확대 기능 포함 버전 (영상 클릭 시 모달)

환경변수로 선택하거나, 기본적으로 basic 버전이 사용됩니다.
"""

# =========================================================
# 1) 표준 라이브러리
# =========================================================
import os
import time
import threading
import queue
import sqlite3
from pathlib import Path
from datetime import datetime

# =========================================================
# 2) 외부 라이브러리
# =========================================================
import cv2
import numpy as np
from flask import Flask, Response, render_template, jsonify, request

# =========================================================
# 3) 환경변수 설정
# =========================================================
WEB1_DEVICE = int(os.getenv("WEB1_DEVICE", "0"))
WEB2_DEVICE = int(os.getenv("WEB2_DEVICE", "4"))

WEB_WIDTH  = int(os.getenv("WEB_WIDTH", "640"))
WEB_HEIGHT = int(os.getenv("WEB_HEIGHT", "360"))
WEB_FPS    = int(os.getenv("WEB_FPS", "15"))

WEB1_CLASS_TOPIC   = os.getenv("WEB1_CLASS_TOPIC", "/signal_web1_class")
WEB1_SECTION_TOPIC = os.getenv("WEB1_SECTION_TOPIC", "/signal_web1_section")
WEB2_CLASS_TOPIC   = os.getenv("WEB2_CLASS_TOPIC", "/signal_web2_class")
WEB2_SECTION_TOPIC = os.getenv("WEB2_SECTION_TOPIC", "/signal_web2_section")

AMR1_IMAGE_TOPIC = os.getenv("AMR1_IMAGE_TOPIC", "/robot5/oakd/rgb/preview/image_raw")
AMR2_IMAGE_TOPIC = os.getenv("AMR2_IMAGE_TOPIC", "/robot4/oakd/rgb/preview/image_raw")

# AMR 도킹 상태 토픽 (Bool 타입)
AMR1_DOCKING_TOPIC = os.getenv("AMR1_DOCKING_TOPIC", "/robot5/dock_status")
AMR2_DOCKING_TOPIC = os.getenv("AMR2_DOCKING_TOPIC", "/robot4/dock_status")

DB_PATH = Path(os.getenv("DB_PATH", "/home/rokey/rokey_ws/src/to_students-main/day5/0122/detect_events.db"))

# HTML 버전 선택 (basic 또는 modal)
HTML_VERSION = os.getenv("HTML_VERSION", "basic").lower()

JPEG_QUALITY = int(os.getenv("JPEG_QUALITY", "75"))
IDLE_SLEEP   = float(os.getenv("IDLE_SLEEP", "0.03"))
POLL_SEC     = float(os.getenv("POLL_SEC", "0.2"))

# YOLO 모델(웹캠/AMR 분리)
YOLO_WEB_MODEL_PATH = os.getenv("YOLO_WEB_MODEL_PATH", "/home/rokey/rokey_ws/src/multi_webcam_monitor/best.pt")
YOLO_AMR_MODEL_PATH = os.getenv("YOLO_AMR_MODEL_PATH", "/home/rokey/rokey_ws/src/multi_webcam_monitor/amr_best.pt")  # 기본값은 예시

YOLO_CONF         = float(os.getenv("YOLO_CONF", "0.5"))
YOLO_INFERENCE_HZ = float(os.getenv("YOLO_INFERENCE_HZ", "5"))
YOLO_TARGET_CLASSES = os.getenv("YOLO_TARGET_CLASSES", "0,1,2")

PUBLISH_NONE      = os.getenv("PUBLISH_NONE", "true").lower() in ("1","true","yes","y","on")
PUBLISH_ALWAYS    = os.getenv("PUBLISH_ALWAYS", "false").lower() in ("1","true","yes","y","on")
FORCE_PUBLISH_HZ  = float(os.getenv("FORCE_PUBLISH_HZ", "2.0"))
LOG_HEARTBEAT_SEC = float(os.getenv("LOG_HEARTBEAT_SEC", "3.0"))

# 웹캠 섹션이 "유효"하다고 인정할 TTL
WEB_SECTION_TTL_SEC = float(os.getenv("WEB_SECTION_TTL_SEC", "30.0"))

# 섹션 그리드(웹캠에서만 계산)
GRID_COLS = int(os.getenv("GRID_COLS", "4"))
GRID_ROWS = int(os.getenv("GRID_ROWS", "4"))

# 오버레이(격자 OFF 기본)
OVERLAY_GRID = os.getenv("OVERLAY_GRID", "false").lower() in ("1","true","yes","y","on")
OVERLAY_BOX  = os.getenv("OVERLAY_BOX",  "true").lower() in ("1","true","yes","y","on")
OVERLAY_TEXT = os.getenv("OVERLAY_TEXT", "true").lower() in ("1","true","yes","y","on")

# =========================================================
# 4) 클래스 이름 매핑
# =========================================================
CLASS_NAME = {0: "컵", 1: "균열", 2: "물"}

# =========================================================
# 5) DB: detect_events(AMR 이벤트) + webcam_state(웹캠 최신 섹션 상태)
# =========================================================
def init_db():
    DB_PATH.parent.mkdir(parents=True, exist_ok=True)
    conn = sqlite3.connect(str(DB_PATH))
    cur = conn.cursor()

    # AMR 이벤트 테이블
    cur.execute(
        """
        CREATE TABLE IF NOT EXISTS detect_events (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            robot TEXT NOT NULL,
            class_id INTEGER NOT NULL,
            name TEXT NOT NULL,
            ts TEXT NOT NULL
        );
        """
    )

    # 기존 DB 호환: section_id 컬럼 없으면 추가
    cur.execute("PRAGMA table_info(detect_events);")
    cols = [r[1] for r in cur.fetchall()]
    if "section_id" not in cols:
        cur.execute("ALTER TABLE detect_events ADD COLUMN section_id INTEGER DEFAULT 0;")
        print("[DB] detect_events에 section_id 컬럼 추가 완료")

    # 웹캠 최신 상태 테이블(섹션만 중요)
    cur.execute(
        """
        CREATE TABLE IF NOT EXISTS webcam_state (
            cam TEXT PRIMARY KEY,
            class_id INTEGER NOT NULL,
            section_id INTEGER NOT NULL,
            ts TEXT NOT NULL
        );
        """
    )

    # 초기 2행(web1/web2) 보장
    now = datetime.now().isoformat(timespec="seconds")
    for cam in ("web1", "web2"):
        cur.execute(
            "INSERT OR IGNORE INTO webcam_state(cam, class_id, section_id, ts) VALUES (?,?,?,?)",
            (cam, 0, 0, now)
        )

    conn.commit()
    conn.close()
    print(f"[DB] 준비 완료: {DB_PATH.resolve()}")

def upsert_webcam_section(cam: str, section_id: int):
    """
    웹캠 최신 섹션만 저장합니다.
    - class_id는 '저장하지 말라'는 요구에 맞춰 의미 없는 값(0)으로만 기록합니다.
      (테이블 구조는 호환 때문에 유지)
    """
    ts = datetime.now().isoformat(timespec="seconds")
    conn = sqlite3.connect(str(DB_PATH))
    cur = conn.cursor()
    cur.execute(
        """
        INSERT INTO webcam_state(cam, class_id, section_id, ts)
        VALUES (?,?,?,?)
        ON CONFLICT(cam) DO UPDATE SET
          class_id=excluded.class_id,
          section_id=excluded.section_id,
          ts=excluded.ts
        """,
        (cam, 0, int(section_id), ts)
    )
    conn.commit()
    conn.close()

def get_webcam_section(cam: str):
    """
    cam(web1/web2)의 최신 섹션을 가져옵니다.
    - 너무 오래됐으면(sec=0으로 처리)
    반환: (sec, age_sec)
    """
    conn = sqlite3.connect(str(DB_PATH))
    cur = conn.cursor()
    cur.execute("SELECT section_id, ts FROM webcam_state WHERE cam=?", (cam,))
    row = cur.fetchone()
    conn.close()

    if not row:
        return 0, None

    sec, ts = row
    try:
        t = datetime.fromisoformat(ts)
        age = (datetime.now() - t).total_seconds()
    except Exception:
        return int(sec), None

    if age > WEB_SECTION_TTL_SEC:
        return 0, age
    return int(sec), age

def delete_web_events():
    """
    detect_events 테이블에서 robot이 'web1' 또는 'web2'인 이벤트를 모두 삭제합니다.
    (웹캠 이벤트는 저장하지 않기로 함)
    """
    conn = sqlite3.connect(str(DB_PATH))
    cur = conn.cursor()
    cur.execute("DELETE FROM detect_events WHERE robot IN ('web1', 'web2')")
    deleted_count = cur.rowcount
    conn.commit()
    conn.close()
    if deleted_count > 0:
        print(f"[DB] Web 이벤트 {deleted_count}개 삭제 완료")
    return deleted_count

def save_amr_event(robot: str, class_id: int, section_id: int):
    name = CLASS_NAME.get(int(class_id), "알수없음")
    ts = datetime.now().isoformat(timespec="seconds")
    conn = sqlite3.connect(str(DB_PATH))
    cur = conn.cursor()
    cur.execute(
        "INSERT INTO detect_events(robot, class_id, name, ts, section_id) VALUES (?,?,?,?,?)",
        (robot, int(class_id), name, ts, int(section_id)),
    )
    conn.commit()
    conn.close()

def fetch_latest_event(robot: str):
    conn = sqlite3.connect(str(DB_PATH))
    cur = conn.cursor()
    try:
        cur.execute(
            "SELECT class_id, name, ts, COALESCE(section_id,0) FROM detect_events WHERE robot=? ORDER BY id DESC LIMIT 1",
            (robot,),
        )
        return cur.fetchone()
    finally:
        conn.close()

def fetch_recent_events(limit: int):
    conn = sqlite3.connect(str(DB_PATH))
    cur = conn.cursor()
    cur.execute(
        "SELECT id, robot, class_id, name, ts, COALESCE(section_id,0) FROM detect_events ORDER BY id DESC LIMIT ?",
        (int(limit),),
    )
    rows = cur.fetchall()
    conn.close()
    return rows

# =========================================================
# 6) MJPEG 최신 프레임 저장소 + AMR 도킹 상태
# =========================================================
_latest = {"web1": (None, 0), "web2": (None, 0), "amr1": (None, 0), "amr2": (None, 0)}
_docking_state = {"amr1": None, "amr2": None}  # True=도킹, False=언도킹, None=알 수 없음
_lock = threading.Lock()

def set_latest(key: str, jpg_bytes: bytes):
    with _lock:
        _, s = _latest[key]
        _latest[key] = (jpg_bytes, s + 1)

def get_latest(key: str):
    with _lock:
        return _latest[key]

def set_docking_state(key: str, is_docked: bool):
    """AMR 도킹 상태 업데이트"""
    with _lock:
        _docking_state[key] = is_docked

def get_docking_state(key: str):
    """AMR 도킹 상태 조회"""
    with _lock:
        return _docking_state.get(key)

def enc_jpg(bgr: np.ndarray):
    ok, buf = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
    return buf.tobytes() if ok else None

# =========================================================
# 7) 섹션(GridConfig) - 웹캠에서만 사용
# =========================================================
try:
    from section_utils import GridConfig
except Exception:
    from dataclasses import dataclass
    @dataclass(frozen=True)
    class GridConfig:
        cols: int = 4
        rows: int = 4
        def section_of(self, x: float, y: float, width: int, height: int) -> int:
            if width <= 0 or height <= 0:
                return 0
            x = max(0.0, min(float(x), float(width - 1)))
            y = max(0.0, min(float(y), float(height - 1)))
            col = int(x / (width / self.cols))
            row = int(y / (height / self.rows))
            col = max(0, min(self.cols - 1, col))
            row = max(0, min(self.rows - 1, row))
            return row * self.cols + col + 1

GRID = GridConfig(cols=GRID_COLS, rows=GRID_ROWS)

def draw_grid(img: np.ndarray, cols: int, rows: int):
    h, w = img.shape[:2]
    for c in range(1, cols):
        x = int(w * c / cols)
        cv2.line(img, (x, 0), (x, h-1), (0,255,0), 1)
    for r in range(1, rows):
        y = int(h * r / rows)
        cv2.line(img, (0, y), (w-1, y), (0,255,0), 1)

# =========================================================
# 8) YOLO 유틸
# =========================================================
def _parse_target_classes(s: str):
    out = set()
    for tok in s.split(","):
        tok = tok.strip()
        if not tok:
            continue
        try:
            out.add(int(tok))
        except Exception:
            pass
    return out

TARGET_CLASSES = _parse_target_classes(YOLO_TARGET_CLASSES)

def best_detection_with_bbox(model, frame):
    """
    YOLO 결과 중 conf 가장 큰 1개만 선택
    반환: (class_id, bbox_xyxy or None)
    """
    try:
        results = model.predict(frame, verbose=False, conf=YOLO_CONF)
        if not results:
            return 0, None
        r0 = results[0]
        if r0.boxes is None or len(r0.boxes) == 0:
            return 0, None

        best = None
        best_conf = -1.0
        for b in r0.boxes:
            cls = int(b.cls.item())
            conf = float(b.conf.item())
            if TARGET_CLASSES and (cls not in TARGET_CLASSES):
                continue
            if conf > best_conf:
                xyxy = b.xyxy[0].tolist()
                best = (cls, conf, xyxy)
                best_conf = conf

        if best is None:
            return 0, None

        cls, conf, xyxy = best
        x1, y1, x2, y2 = xyxy
        return int(cls), (int(x1), int(y1), int(x2), int(y2))
    except Exception:
        return 0, None

def overlay_boxes(frame_bgr: np.ndarray, key: str, cls: int, sec: int, bbox, sec_source: str):
    """
    화면 위에:
    - bbox(있으면 사각형)
    - cls/sec 텍스트
    를 그려서 웹에서 즉시 확인 가능하게 함
    (웹캠 전용)
    """
    vis = frame_bgr.copy()

    if OVERLAY_GRID:
        draw_grid(vis, GRID_COLS, GRID_ROWS)

    if OVERLAY_BOX and bbox is not None:
        x1, y1, x2, y2 = bbox
        cv2.rectangle(vis, (x1,y1), (x2,y2), (0,255,0), 2)

    if OVERLAY_TEXT:
        name = CLASS_NAME.get(int(cls), "알수없음")
        txt = f"{key} cls={cls}({name}) sec={sec} (from {sec_source})"
        cv2.putText(vis, txt, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0,255,0), 2)

    return vis

def overlay_amr_boxes(frame_bgr: np.ndarray, bbox):
    """
    AMR 전용 오버레이: 바운딩 박스만 표시 (텍스트 없음)
    """
    vis = frame_bgr.copy()
    
    # 바운딩 박스만 그리기
    if OVERLAY_BOX and bbox is not None:
        x1, y1, x2, y2 = bbox
        cv2.rectangle(vis, (x1,y1), (x2,y2), (0,255,0), 2)
    
    return vis

# =========================================================
# 9) 웹캠 캡처 스레드
# =========================================================
def _open_cam(dev_index: int):
    dev_path = f"/dev/video{dev_index}"
    cap = cv2.VideoCapture(dev_path, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap = cv2.VideoCapture(dev_index)
    if not cap.isOpened():
        raise RuntimeError(f"웹캠 오픈 실패: {dev_path} (또는 index {dev_index})")
    try:
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    except Exception:
        pass
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WEB_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, WEB_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, WEB_FPS)
    return cap

class WebcamCapture(threading.Thread):
    """
    웹캠의 역할:
    - YOLO로 bbox+class를 찾는다(웹에서 박스 표시 + 토픽 발행용)
    - bbox 중심으로 section을 계산한다(섹션은 '웹캠에서만' 결정)
    - 최신 section을 DB(webcam_state)에 저장한다 (AMR이 이 값을 붙여서 저장)
    """
    def __init__(self, cam_key: str, dev_index: int, yolo_model, out_q: queue.Queue):
        super().__init__(daemon=True)
        self.cam_key = cam_key
        self.dev_index = dev_index
        self.model = yolo_model
        self.out_q = out_q

        self.last_cls = 0
        self.last_sec = 0
        self.last_bbox = None
        self._last_infer_t = 0.0

    def run(self):
        cap = None
        period = max(IDLE_SLEEP, 1.0 / max(1, WEB_FPS))
        infer_period = 1.0 / max(0.1, YOLO_INFERENCE_HZ)

        while True:
            if cap is None:
                try:
                    cap = _open_cam(self.dev_index)
                    print(f"[CAM] {self.cam_key}: 오픈 성공 /dev/video{self.dev_index}")
                except Exception as e:
                    print(f"[CAM] {self.cam_key}: 오픈 실패: {e}")
                    time.sleep(1.0)
                    continue

            ok, frm = cap.read()
            if not ok or frm is None:
                print(f"[CAM] {self.cam_key}: 읽기 실패 -> 재오픈")
                try:
                    cap.release()
                except Exception:
                    pass
                cap = None
                time.sleep(0.2)
                continue

            now = time.time()

            if self.model is not None and (now - self._last_infer_t) >= infer_period:
                self._last_infer_t = now
                cls, bbox = best_detection_with_bbox(self.model, frm)

                # 웹캠에서만 section 계산
                sec = 0
                if bbox is not None:
                    h, w = frm.shape[:2]
                    x1, y1, x2, y2 = bbox
                    cx = (x1 + x2) / 2.0
                    cy = (y1 + y2) / 2.0
                    sec = GRID.section_of(cx, cy, w, h)

                self.last_cls, self.last_sec, self.last_bbox = cls, sec, bbox

                # 웹캠 최신 섹션 저장(AMR이 참조)
                upsert_webcam_section(self.cam_key, sec)

                # 토픽 publish 큐로 전달(webcam 결과 -> ROS publish)
                try:
                    self.out_q.put_nowait((self.cam_key, int(cls), int(sec), now))
                except queue.Full:
                    pass

            # 웹 화면 오버레이: 웹캠은 sec_source가 자기 자신
            vis = overlay_boxes(frm, self.cam_key, self.last_cls, self.last_sec, self.last_bbox, self.cam_key)
            j = enc_jpg(vis)
            if j:
                set_latest(self.cam_key, j)

            time.sleep(period)

# =========================================================
# 10) ROS Runner
# =========================================================
class RosRunner(threading.Thread):
    """
    - 웹캠 결과를 ROS 토픽으로 publish
    - AMR 영상 토픽을 subscribe
    - AMR 영상은 AMR용 YOLO로 박스를 그리고,
      DB 저장할 때는 section을 고정 매핑(web1/web2)에서 가져와 붙임
    """
    def __init__(self, det_q: queue.Queue, yolo_model_amr):
        super().__init__(daemon=True)
        self.det_q = det_q
        self.model_amr = yolo_model_amr

    def run(self):
        try:
            import rclpy
            from rclpy.node import Node
            from rclpy.executors import MultiThreadedExecutor
            from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
            from std_msgs.msg import Int32, Bool
            from sensor_msgs.msg import CompressedImage, Image
        except Exception as e:
            print(f"[ROS] import 실패: {e}")
            return

        def to_bgr(m: Image):
            try:
                h, w = int(m.height), int(m.width)
                step = int(m.step) if hasattr(m, "step") else w * 3
                buf = np.frombuffer(m.data, dtype=np.uint8)
                row_bytes = step if step > 0 else w * 3
                total = h * row_bytes
                if buf.size < total:
                    return None
                buf = buf[:total].reshape(h, row_bytes)[:, : w * 3].reshape(h, w, 3)
                enc = (m.encoding or "").lower()
                return cv2.cvtColor(buf, cv2.COLOR_RGB2BGR) if enc.startswith("rgb") else buf
            except Exception:
                return None

        class PubNode(Node):
            def __init__(self):
                super().__init__("webcam_web_dashboard_publisher")
                qos = QoSProfile(
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.VOLATILE,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=1,
                )
                self.pub_w1_class = self.create_publisher(Int32, WEB1_CLASS_TOPIC.lstrip("/"), qos)
                self.pub_w1_sec   = self.create_publisher(Int32, WEB1_SECTION_TOPIC.lstrip("/"), qos)
                self.pub_w2_class = self.create_publisher(Int32, WEB2_CLASS_TOPIC.lstrip("/"), qos)
                self.pub_w2_sec   = self.create_publisher(Int32, WEB2_SECTION_TOPIC.lstrip("/"), qos)

                self.last_pub = {"web1": (None, None), "web2": (None, None)}
                self.last_force_t = {"web1": 0.0, "web2": 0.0}
                self.last_hb = 0.0

            def publish_pair(self, key: str, cls: int, sec: int):
                msg_c = Int32(); msg_s = Int32()
                msg_c.data = int(cls); msg_s.data = int(sec)
                if key == "web1":
                    self.pub_w1_class.publish(msg_c)
                    self.pub_w1_sec.publish(msg_s)
                else:
                    self.pub_w2_class.publish(msg_c)
                    self.pub_w2_sec.publish(msg_s)

        class SubNode(Node):
            def __init__(self, model_amr):
                super().__init__("monitoring_web_db")
                self.model_amr = model_amr

                self._infer_period = 1.0 / max(0.1, YOLO_INFERENCE_HZ)
                self._last_infer_t = {"amr1": 0.0, "amr2": 0.0}

                # 같은 (cls, sec) 연속이면 1번만 저장
                self._last_event = {"amr1": (None, None), "amr2": (None, None)}
                self._saved_once = {"amr1": False, "amr2": False}

                self._sub_image_auto("amr1", AMR1_IMAGE_TOPIC, CompressedImage, Image)
                self._sub_image_auto("amr2", AMR2_IMAGE_TOPIC, CompressedImage, Image)

                # 도킹 상태 구독 (실패해도 영상은 정상 작동)
                try:
                    self.create_subscription(Bool, AMR1_DOCKING_TOPIC, lambda m: self._on_docking("amr1", m), 10)
                    self.create_subscription(Bool, AMR2_DOCKING_TOPIC, lambda m: self._on_docking("amr2", m), 10)
                    self.get_logger().info(f"[DOCKING] {AMR1_DOCKING_TOPIC}, {AMR2_DOCKING_TOPIC}")
                except Exception as e:
                    self.get_logger().warning(f"[DOCKING] 구독 실패 (영상은 정상 작동): {e}")

                self.get_logger().info("[ROS-SUB] AMR 영상 구독 시작")
                self.get_logger().info(f"[AMR YOLO] {YOLO_AMR_MODEL_PATH}")
                self.get_logger().info("[SEC MAP] amr1<-web1, amr2<-web2 (고정)")

            def _sub_image_auto(self, key: str, topic: str, CompressedImageT, ImageT):
                if topic.endswith("/compressed") or topic.endswith("compressed"):
                    self.create_subscription(CompressedImageT, topic, lambda m: self._on_amr_compressed(key, m), 10)
                else:
                    self.create_subscription(ImageT, topic, lambda m: self._on_amr_image(key, m), 10)

            def _on_amr_compressed(self, key: str, m: CompressedImage):
                try:
                    arr = np.frombuffer(m.data, dtype=np.uint8)
                    bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                    if bgr is None:
                        return
                    self._process_amr_frame(key, bgr)
                except Exception:
                    return

            def _on_amr_image(self, key: str, m: Image):
                bgr = to_bgr(m)
                if bgr is None:
                    return
                self._process_amr_frame(key, bgr)

            def _mapped_webcam(self, amr_key: str) -> str:
                # ✅ 고정 매핑 규칙
                return "web1" if amr_key == "amr1" else "web2"

            def _on_docking(self, key: str, m: Bool):
                """도킹 상태 업데이트 (True=도킹, False=언도킹)"""
                try:
                    set_docking_state(key, m.data)
                except Exception as e:
                    self.get_logger().error(f"[DOCKING ERROR] {key}: {e}")

            def _process_amr_frame(self, key: str, bgr: np.ndarray):
                now = time.time()

                cls = 0
                bbox = None

                # (1) AMR YOLO(주기 제한)
                if self.model_amr is not None:
                    if (now - self._last_infer_t[key]) >= self._infer_period:
                        self._last_infer_t[key] = now
                        cls, bbox = best_detection_with_bbox(self.model_amr, bgr)

                # (2) section은 고정 매핑된 웹캠에서 가져온다
                wcam = self._mapped_webcam(key)
                sec, age = get_webcam_section(wcam)

                # (3) 웹 화면에 표시: AMR은 바운딩 박스만 표시
                vis = overlay_amr_boxes(bgr, bbox)
                j = enc_jpg(vis)
                if j:
                    set_latest(key, j)

                # (4) DB 저장: AMR만 저장, 감지 없으면 저장 안 함
                if cls != 0:
                    last_cls, last_sec = self._last_event[key]
                    if (last_cls, last_sec) != (cls, sec):
                        self._saved_once[key] = False
                        self._last_event[key] = (cls, sec)

                    if not self._saved_once[key]:
                        save_amr_event(key, cls, sec)
                        self._saved_once[key] = True
                        self.get_logger().info(f"[DB] AMR 저장: {key} cls={cls} sec={sec} (from {wcam}, age={age})")

        rclpy.init()

        pub = PubNode()
        sub = SubNode(self.model_amr)

        ex = MultiThreadedExecutor(num_threads=3)
        ex.add_node(pub)
        ex.add_node(sub)

        try:
            while rclpy.ok():
                ex.spin_once(timeout_sec=0.05)
                now = time.time()

                # 하트비트(토픽 연결 확인용)
                if (now - pub.last_hb) >= max(0.5, LOG_HEARTBEAT_SEC):
                    pub.last_hb = now
                    try:
                        c1 = pub.pub_w1_class.get_subscription_count()
                        c2 = pub.pub_w2_class.get_subscription_count()
                    except Exception:
                        c1 = c2 = -1
                    pub.get_logger().info(f"[HB] subs w1_class={c1} w2_class={c2}")

                # 웹캠 publish 큐 처리
                drained = 0
                while drained < 10:
                    try:
                        key, cls, sec, _t = self.det_q.get_nowait()
                    except queue.Empty:
                        break
                    drained += 1

                    if key not in ("web1", "web2"):
                        continue

                    should_publish = not (cls == 0 and not PUBLISH_NONE)

                    force_period = (1.0 / FORCE_PUBLISH_HZ) if FORCE_PUBLISH_HZ > 0 else None
                    if force_period is not None and (now - pub.last_force_t[key]) >= force_period:
                        if should_publish:
                            pub.publish_pair(key, cls, sec)
                            pub.last_pub[key] = (cls, sec)
                        pub.last_force_t[key] = now
                        continue

                    if should_publish and (PUBLISH_ALWAYS or pub.last_pub[key] != (cls, sec)):
                        pub.publish_pair(key, cls, sec)
                        pub.last_pub[key] = (cls, sec)

        except KeyboardInterrupt:
            pass
        finally:
            try:
                ex.shutdown()
            except Exception:
                pass
            try:
                pub.destroy_node()
                sub.destroy_node()
            except Exception:
                pass
            try:
                rclpy.shutdown()
            except Exception:
                pass

# =========================================================
# 11) Flask
# =========================================================
app = Flask(__name__, template_folder=os.path.dirname(__file__))

def mjpeg_stream(name: str):
    last_seq = -1
    while True:
        jpg, seq = get_latest(name)
        if jpg is None or seq == last_seq:
            time.sleep(IDLE_SLEEP)
            continue
        last_seq = seq
        yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n"

@app.route("/")
def dashboard():
    """
    대시보드 메인 페이지
    환경변수 HTML_VERSION에 따라 다른 HTML 파일 로드
    - basic: dashboard_v8a_basic.html (기본값)
    - modal: dashboard_v8b_modal_new.html (영상 확대 기능)
    """
    # HTML 파일명 결정
    if HTML_VERSION == "modal":
        html_filename = "dashboard_v8b_modal_new.html"
    else:
        html_filename = "dashboard_v8a_basic.html"
    
    # HTML 파일을 직접 읽어서 반환 (템플릿 로딩 문제 해결)
    html_path = os.path.join(os.path.dirname(__file__), html_filename)
    
    if os.path.exists(html_path):
        with open(html_path, 'r', encoding='utf-8') as f:
            html_content = f.read()
            # Jinja2 템플릿 변수 치환
            html_content = html_content.replace('{{ poll_sec | safe }}', str(POLL_SEC))
            return html_content
    else:
        # HTML 파일이 없으면 에러 메시지 표시
        return f"""
        <h1>❌ HTML 파일을 찾을 수 없습니다</h1>
        <p><b>찾으려고 한 파일:</b> {html_path}</p>
        <p><b>현재 HTML_VERSION:</b> {HTML_VERSION}</p>
        <hr>
        <h3>해결 방법:</h3>
        <ol>
            <li>Python 파일과 같은 디렉토리에 HTML 파일을 배치하세요</li>
            <li>파일명 확인:
                <ul>
                    <li>기본 버전: <code>dashboard_v8a_basic.html</code></li>
                    <li>확대 기능 포함: <code>dashboard_v8b_modal.html</code></li>
                </ul>
            </li>
            <li>환경변수 설정 (선택):
                <ul>
                    <li><code>export HTML_VERSION=basic</code> (기본값)</li>
                    <li><code>export HTML_VERSION=modal</code> (확대 기능)</li>
                </ul>
            </li>
        </ol>
        """, 500


@app.route("/video/<name>")
def video(name):
    if name not in ("web1", "web2", "amr1", "amr2"):
        return ("not found", 404)
    return Response(mjpeg_stream(name), mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/api/db_status")
def api_db_status():
    now = datetime.now()

    def row_to_item(row):
        if not row:
            return {"object_id": None, "object_name": None, "detected_at": None, "age_sec": None, "section_id": None}
        class_id, name, ts, section_id = row
        try:
            t = datetime.fromisoformat(ts)
            age = (now - t).total_seconds()
        except Exception:
            age = None
        return {
            "object_id": int(class_id),
            "object_name": name,
            "detected_at": ts,
            "age_sec": age,
            "section_id": int(section_id) if section_id is not None else 0,
        }

    # AMR 최신 이벤트 1개씩
    cams = {
        "3": row_to_item(fetch_latest_event("amr1")),
        "4": row_to_item(fetch_latest_event("amr2")),
    }

    # AMR 도킹 상태 추가
    cams["amr1_docking"] = get_docking_state("amr1")  # True/False/None
    cams["amr2_docking"] = get_docking_state("amr2")

    # 디버그용: 웹캠 최신 섹션도 같이 반환
    s1, a1 = get_webcam_section("web1")
    s2, a2 = get_webcam_section("web2")
    cams["web1_section"] = {"section_id": s1, "age_sec": a1}
    cams["web2_section"] = {"section_id": s2, "age_sec": a2}

    return jsonify({"server_time": now.isoformat(timespec="seconds"), "cams": cams})

@app.route("/events")
def events_page():
    limit = int(request.args.get("limit", "200"))
    limit = max(1, min(2000, limit))
    rows = fetch_recent_events(limit)

    html = []
    html.append("<!doctype html><html><head><meta charset='utf-8'>")
    html.append("<title>Detect Events</title>")
    html.append("""
    <style>
      body{font-family:Arial, sans-serif; margin:20px;}
      h2{margin:0 0 10px 0;}
      table{border-collapse:collapse; width:100%;}
      th,td{border:1px solid #ddd; padding:8px; text-align:left;}
      th{background:#f3f3f3;}
      tr:nth-child(even){background:#fafafa;}
      .muted{color:#666; font-size:12px;}
      .btn{display:inline-block; padding:6px 10px; border:1px solid #aaa; border-radius:6px; text-decoration:none; color:#111; margin-right:6px;}
    </style>
    """)
    html.append("</head><body>")
    html.append(f"<h2>AMR 이벤트 목록(최근 {limit}개)</h2>")
    html.append("<div class='muted'>새로고침(F5)하면 갱신됩니다. (Web 이벤트는 저장하지 않음)</div><br>")
    html.append("<a class='btn' href='/'>대시보드</a><br><br>")
    html.append("<table><thead><tr>")
    html.append("<th>ID</th><th>Robot</th><th>객체 이름</th><th>Section(웹캠)</th><th>Timestamp</th>")
    html.append("</tr></thead><tbody>")

    for (eid, robot, class_id, name, ts, section_id) in rows:
        html.append("<tr>")
        html.append(f"<td>{eid}</td><td>{robot}</td><td>{name}</td><td>{section_id}</td><td>{ts}</td>")
        html.append("</tr>")

    html.append("</tbody></table>")
    html.append("</body></html>")
    return "".join(html)

# =========================================================
# 12) main
# =========================================================
def main():
    init_db()
    
    # Web 관련 이벤트 삭제 (웹캠 이벤트는 저장하지 않음)
    delete_web_events()

    # YOLO 로드(웹캠/AMR 분리)
    yolo_web1 = None
    yolo_web2 = None
    yolo_amr  = None

    try:
        from ultralytics import YOLO

        if os.path.exists(YOLO_WEB_MODEL_PATH):
            yolo_web1 = YOLO(YOLO_WEB_MODEL_PATH)
            yolo_web2 = YOLO(YOLO_WEB_MODEL_PATH)
            print(f"[YOLO-WEB] 로드 성공: {YOLO_WEB_MODEL_PATH}")
        else:
            print(f"[YOLO-WEB] 파일 없음: {YOLO_WEB_MODEL_PATH} -> 웹캠 YOLO 비활성화")

        if os.path.exists(YOLO_AMR_MODEL_PATH):
            yolo_amr = YOLO(YOLO_AMR_MODEL_PATH)
            print(f"[YOLO-AMR] 로드 성공: {YOLO_AMR_MODEL_PATH}")
        else:
            print(f"[YOLO-AMR] 파일 없음: {YOLO_AMR_MODEL_PATH} -> AMR YOLO 비활성화")

    except Exception as e:
        print(f"[YOLO] import/load 실패({e}) -> YOLO 전체 비활성화")

    # 웹캠 결과 큐(ROS publish로 전달)
    det_q: queue.Queue = queue.Queue(maxsize=200)

    # 웹캠 스레드 시작
    WebcamCapture("web1", WEB1_DEVICE, yolo_web1, det_q).start()
    WebcamCapture("web2", WEB2_DEVICE, yolo_web2, det_q).start()

    # ROS 스레드 시작
    RosRunner(det_q, yolo_amr).start()

    # Flask 시작
    app.run(host="0.0.0.0", port=5000, debug=False, use_reloader=False, threaded=True)

if __name__ == "__main__":
    main()
