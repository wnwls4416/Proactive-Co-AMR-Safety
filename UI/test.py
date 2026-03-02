#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, time, threading, sqlite3
from datetime import datetime

import cv2, numpy as np
from flask import Flask, Response, render_template, jsonify, make_response

# =========================================================
# 1) 설정값(여기만 수정하면 나머지는 그대로 동작)
# =========================================================

# USB 웹캠(모니터링 화면용)
WEB1_DEVICE = int(os.getenv("WEB1_DEVICE", "0"))
WEB2_DEVICE = int(os.getenv("WEB2_DEVICE", "4"))

WEB_WIDTH   = int(os.getenv("WEB_WIDTH", "640"))
WEB_HEIGHT  = int(os.getenv("WEB_HEIGHT", "360"))
WEB_FPS     = int(os.getenv("WEB_FPS", "15"))

# AMR 영상 토픽(나중에 AMR 코드와 대조해서 "문자열만" 수정)
AMR1_IMAGE_TOPIC = os.getenv("AMR1_IMAGE_TOPIC", "/robot5/oakd/rgb/preview/image_raw")
AMR2_IMAGE_TOPIC = os.getenv("AMR2_IMAGE_TOPIC", "/robot4/oakd/rgb/preview/image_raw")

# AMR 클래스 토픽(섹션 없음 버전: class(Int32)만 받음)
# - 여기만 수정해도 페이지는 정상 실행되게 만들었음
AMR1_CLASS_TOPIC = os.getenv("AMR1_CLASS_TOPIC", "/signal_amr1_class")
AMR2_CLASS_TOPIC = os.getenv("AMR2_CLASS_TOPIC", "/signal_amr2_class")

# MJPEG 품질/루프
JPEG_QUALITY = int(os.getenv("JPEG_QUALITY", "75"))
IDLE_SLEEP   = float(os.getenv("IDLE_SLEEP", "0.03"))

########################################################

DB_PATH = "/home/rokey/rokey_ws/src/to_students-main/day5/0122/mydatabase.db"
#/home/rokey/rokey_ws/src/to_students-main/day5/0122/mydatabase.db

# 상태 페이지 자동 갱신(초)
POLL_SEC = float(os.getenv("POLL_SEC", "0.2"))

# DB(기존 DB 파일 그대로 쌓기)
# DB(기존 DB 파일 그대로 쌓기)
# - DB_PATH가 None/빈값/"None"로 들어오는 경우를 방지하기 위해 보정
DEFAULT_DB_PATH = "/home/rokey/rokey_ws/src/to_students-main/day5/mydatabase.db"
DB_PATH = os.getenv("DB_PATH")  # 환경변수로 주면 그걸 우선

if DB_PATH is None or DB_PATH.strip() == "" or DB_PATH.strip().lower() == "none":
    DB_PATH = DEFAULT_DB_PATH

ENABLE_DB_LOG = (os.getenv("ENABLE_DB_LOG", "1") == "1")          # DB 저장 기능 on/off
SAVE_ONLY_ON_CHANGE = (os.getenv("SAVE_ONLY_ON_CHANGE", "1") == "1")  # 값이 바뀔 때만 저장
DEFAULT_LOG_LIMIT = int(os.getenv("DEFAULT_LOG_LIMIT", "200"))


# =========================================================
# 2) 프레임 저장소(웹캠/AMR 공용) + 기본 유틸
# =========================================================

_latest = {
    "web1": (None, 0),
    "web2": (None, 0),
    "amr1": (None, 0),
    "amr2": (None, 0),
}  # 카메라별 최신 jpeg, 버전(0/1) 갱신용

_lock = threading.Lock()  # 멀티스레드 안전

def set_latest(k, jpg_bytes):  # 카메라별 최신 프레임 저장(덮어쓰기)
    with _lock:
        _, s = _latest[k]
        _latest[k] = (jpg_bytes, s + 1)

def get_latest(k):  # 최신 프레임 읽기
    with _lock:
        return _latest[k]

def enc_jpg(bgr):  # OpenCV BGR -> JPEG bytes
    ok, buf = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
    return buf.tobytes() if ok else None

# =========================================================
# 3) USB 웹캠 스레드(그대로 유지)
# =========================================================

class Cam(threading.Thread):  # USB 웹캠 -> JPEG로 압축 -> 최신 프레임 저장
    def __init__(self, k, dev):
        super().__init__(daemon=True)
        self.k = k
        self.dev = dev

    def run(self):
        cap = cv2.VideoCapture(self.dev)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, WEB_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, WEB_HEIGHT)
        cap.set(cv2.CAP_PROP_FPS, WEB_FPS)
        period = max(IDLE_SLEEP, 1.0 / max(1, WEB_FPS))

        while True:
            ok, frm = cap.read()
            if ok and frm is not None:
                j = enc_jpg(frm)
                if j:
                    set_latest(self.k, j)
            time.sleep(period)

# =========================================================
# 4) AMR 상태값(클래스만) + DB 저장(섹션 없음 버전)
# =========================================================

_sig_lock = threading.Lock()
_sig = {
    "amr1": {"cls": 0, "rx": 0, "last_rx": "-", "last_saved": None},
    "amr2": {"cls": 0, "rx": 0, "last_rx": "-", "last_saved": None},
}

def db_init():  # 기존 프로그램과 동일한 테이블명 유지
    os.makedirs(os.path.dirname(DB_PATH), exist_ok=True) #db 없을때 생성

    conn = sqlite3.connect(DB_PATH)
    cur = conn.cursor()
    cur.execute("""
    CREATE TABLE IF NOT EXISTS webcam_detection_log (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        cam_id INTEGER NOT NULL,
        object_id INTEGER NOT NULL,
        detected_at TEXT NOT NULL
    );
    """)
    conn.commit()
    conn.close()

def db_insert(cam_id: int, object_id: int):  # 값 기록
    now_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    conn = sqlite3.connect(DB_PATH)
    cur = conn.cursor()
    cur.execute(
        "INSERT INTO webcam_detection_log (cam_id, object_id, detected_at) VALUES (?, ?, ?)",
        (int(cam_id), int(object_id), now_str)
    )
    conn.commit()
    conn.close()

def db_read_logs(limit: int):
    limit = max(1, min(int(limit), 5000))
    conn = sqlite3.connect(DB_PATH)
    cur = conn.cursor()
    cur.execute("""
    SELECT id, cam_id, object_id, detected_at
    FROM webcam_detection_log
    ORDER BY id DESC
    LIMIT ?;
    """, (limit,))
    rows = cur.fetchall()
    conn.close()

    out = []
    for _id, cam_id, object_id, detected_at in rows:
        out.append({
            "id": int(_id),
            "cam_id": int(cam_id),
            "object_id": int(object_id),
            "detected_at": str(detected_at),
        })
    return out

# =========================================================
# 5) ROS 스레드(AMR 영상 + AMR class(Int32) 구독)
# =========================================================

class Ros(threading.Thread):
    def run(self):
        try:
            import rclpy
            from rclpy.node import Node
            from rclpy.executors import MultiThreadedExecutor
            from sensor_msgs.msg import CompressedImage, Image
            from std_msgs.msg import Int32
        except Exception as e:
            print(f"[ROS] import 실패: {e}")
            return

        def to_bgr(m: Image):  # sensor_msgs/Image -> OpenCV BGR
            try:
                h, w = int(m.height), int(m.width)
                step = int(m.step) if hasattr(m, "step") else w * 3
                buf = np.frombuffer(m.data, dtype=np.uint8)
                rb = step if step > 0 else w * 3
                tot = h * rb
                if buf.size < tot:
                    return None
                buf = buf[:tot].reshape(h, rb)[:, :w * 3].reshape(h, w, 3)
                enc = (m.encoding or "").lower()
                return cv2.cvtColor(buf, cv2.COLOR_RGB2BGR) if enc.startswith("rgb") else buf
            except Exception:
                return None

        class N(Node):
            def __init__(self):
                super().__init__("monitoring_web_db")

                # --- AMR 영상 구독(토픽이 /compressed면 CompressedImage로 받음) ---
                self._sub_image_auto("amr1", AMR1_IMAGE_TOPIC)
                self._sub_image_auto("amr2", AMR2_IMAGE_TOPIC)

                # --- AMR class(Int32) 구독(섹션 없음) ---
                if AMR1_CLASS_TOPIC:
                    self.create_subscription(Int32, AMR1_CLASS_TOPIC,
                                             lambda m: self._on_class("amr1", 1, int(m.data)), 10)
                if AMR2_CLASS_TOPIC:
                    self.create_subscription(Int32, AMR2_CLASS_TOPIC,
                                             lambda m: self._on_class("amr2", 2, int(m.data)), 10)

                self.get_logger().info("[ROS] 구독 시작")
                self.get_logger().info(f"[IMG] amr1={AMR1_IMAGE_TOPIC}")
                self.get_logger().info(f"[IMG] amr2={AMR2_IMAGE_TOPIC}")
                self.get_logger().info(f"[CLS] amr1={AMR1_CLASS_TOPIC}")
                self.get_logger().info(f"[CLS] amr2={AMR2_CLASS_TOPIC}")

            def _sub_image_auto(self, key: str, topic: str):
                # topic 끝에 compressed가 붙으면 CompressedImage로 받기(가장 안정적)
                if topic.endswith("/compressed") or topic.endswith("compressed"):
                    self.create_subscription(CompressedImage, topic,
                                             lambda m: set_latest(key, bytes(m.data)), 10)
                else:
                    # raw Image로 받으면 JPEG로 변환해서 넣기
                    if key == "amr1":
                        self.create_subscription(Image, topic, self._on_img1, 10)
                    else:
                        self.create_subscription(Image, topic, self._on_img2, 10)

            def _on_img1(self, m: Image):
                b = to_bgr(m)
                if b is None:
                    return
                j = enc_jpg(b)
                if j:
                    set_latest("amr1", j)

            def _on_img2(self, m: Image):
                b = to_bgr(m)
                if b is None:
                    return
                j = enc_jpg(b)
                if j:
                    set_latest("amr2", j)


##########


            def _on_class(self, key: str, cam_id: int, cls_value: int):
                # 1) 상태 갱신(락 안)
                with _sig_lock:
                    _sig[key]["cls"] = int(cls_value)
                    _sig[key]["rx"] += 1
                    _sig[key]["last_rx"] = datetime.now().strftime("%H:%M:%S")

                    should_save = False
                    if ENABLE_DB_LOG:
                        if not SAVE_ONLY_ON_CHANGE:
                            should_save = True
                        else:
                            last = _sig[key]["last_saved"]
                            if last is None or int(last) != int(cls_value):
                                should_save = True
                        if should_save:
                            _sig[key]["last_saved"] = int(cls_value)

                # 2) DB 저장(락 밖)
                if should_save:
                    try:
                        db_insert(cam_id, int(cls_value))
                    except Exception as e:
                        self.get_logger().error(f"[DB 저장 실패] cam={cam_id}, cls={cls_value}, err={e}")

        rclpy.init()
        n = N()
        ex = MultiThreadedExecutor(num_threads=2)
        ex.add_node(n)
        try:
            ex.spin()
        finally:
            try: ex.shutdown()
            except Exception: pass
            try: n.destroy_node()
            except Exception: pass
            try: rclpy.shutdown()
            except Exception: pass

# =========================================================
# 6) Flask (영상 + 상태 + 로그)
# =========================================================

app = Flask(__name__)  # flask 웹서버 생성

@app.after_request
def no_cache(resp):  # 브라우저 캐시 방지(실시간 갱신 느낌 유지)
    resp.headers["Cache-Control"] = "no-store, no-cache, must-revalidate, max-age=0"
    resp.headers["Pragma"] = "no-cache"
    resp.headers["Expires"] = "0"
    return resp

def mjpeg(k):  # 카메라 하나(key=k)의 MJPEG 스트림 생성
    last = -1
    while True:
        j, s = get_latest(k)
        if j is None or s == last:
            time.sleep(IDLE_SLEEP)
            continue
        last = s
        yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + j + b"\r\n")

@app.route("/")  # 기존 4카메라 대시보드(템플릿 그대로 사용)
def dash():
    return render_template("dashboard_4cam.html")

@app.route("/video/<name>")  # 템플릿에서 /video/web1 같은 방식으로 호출
def vid(name):
    if name not in ("web1", "web2", "amr1", "amr2"):
        return ("not found", 404)
    return Response(mjpeg(name), mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/api/status")  # AMR class 값 실시간 JSON
def api_status():
    with _sig_lock:
        snap = {
            "server_time": datetime.now().strftime("%H:%M:%S"),
            "amr1": {"cls": _sig["amr1"]["cls"], "rx": _sig["amr1"]["rx"], "last_rx": _sig["amr1"]["last_rx"]},
            "amr2": {"cls": _sig["amr2"]["cls"], "rx": _sig["amr2"]["rx"], "last_rx": _sig["amr2"]["last_rx"]},
        }
    return jsonify(snap)

@app.route("/status")  # AMR class 값 보기 페이지(섹션 없음)
def status_page():
    html = f"""
    <html><head><meta charset="utf-8"><title>AMR Status</title>
    <style>
      body {{ font-family: Arial, sans-serif; }}
      table {{ border-collapse: collapse; margin-top: 10px; }}
      th, td {{ border: 1px solid #333; padding: 6px 10px; }}
      th {{ background: #f2f2f2; }}
      .small {{ color: #444; font-size: 14px; margin-top: 8px; }}
      code {{ background:#f6f6f6; padding:2px 4px; }}
    </style>
    </head>
    <body>
      <h2>AMR class(Int32) 실시간 (섹션 없음)</h2>
      <div class="small">
        자동 갱신: {POLL_SEC:.1f}s |
        DB저장: {str(ENABLE_DB_LOG)} (변경시에만 저장={str(SAVE_ONLY_ON_CHANGE)})
      </div>

      <p>
        <a href="/">카메라 대시보드</a> |
        <a href="/logs">DB 로그 보기</a>
      </p>

      <div class="small" id="serverTime">서버시간: -</div>

      <table>
        <tr>
          <th>대상</th><th>class</th><th>rx</th><th>마지막 수신</th>
        </tr>
        <tbody id="rows"><tr><td colspan="4">불러오는 중...</td></tr></tbody>
      </table>

      <div class="small" style="margin-top:12px;">
        토픽 이름은 app.py 상단의 <code>AMR1_CLASS_TOPIC</code>, <code>AMR2_CLASS_TOPIC</code> 문자열만 바꿔도 정상 실행됩니다.
      </div>

      <script>
        async function refresh() {{
          const res = await fetch("/api/status?ts=" + Date.now());
          const data = await res.json();

          document.getElementById("serverTime").innerText = "서버시간: " + (data.server_time || "-");

          const tbody = document.getElementById("rows");
          tbody.innerHTML = "";

          const items = [
            ["amr1", data.amr1],
            ["amr2", data.amr2],
          ];

          for (const [name, x] of items) {{
            const tr = document.createElement("tr");
            tr.innerHTML =
              `<td>${{name}}</td>` +
              `<td>${{x.cls}}</td>` +
              `<td>${{x.rx}}</td>` +
              `<td>${{x.last_rx}}</td>`;
            tbody.appendChild(tr);
          }}
        }}

        refresh();
        setInterval(refresh, {int(POLL_SEC * 1000)});
      </script>
    </body></html>
    """
    return make_response(html)

@app.route("/api/logs")  # DB 로그 JSON
def api_logs():
    if not ENABLE_DB_LOG:
        return jsonify({"error": "DB logging disabled", "items": [], "server_time": datetime.now().strftime("%H:%M:%S")})
    limit = int(os.getenv("LOG_LIMIT", str(DEFAULT_LOG_LIMIT)))
    return jsonify({"limit": limit, "items": db_read_logs(limit), "server_time": datetime.now().strftime("%H:%M:%S")})

@app.route("/logs")  # DB 로그 페이지(최근 N개)
def logs_page():
    if not ENABLE_DB_LOG:
        return make_response("""
        <h2>DB 로그 기능이 꺼져있습니다(ENABLE_DB_LOG=0)</h2>
        <a href="/status">Status로 돌아가기</a>
        """)

    limit = DEFAULT_LOG_LIMIT
    html = f"""
    <html><head><meta charset="utf-8"><title>DB Logs</title>
    <style>
      body {{ font-family: Arial, sans-serif; }}
      table {{ border-collapse: collapse; margin-top: 10px; }}
      th, td {{ border: 1px solid #333; padding: 6px 10px; }}
      th {{ background: #f2f2f2; }}
      .small {{ color: #444; font-size: 14px; margin-top: 8px; }}
      code {{ background:#f6f6f6; padding:2px 4px; }}
    </style>
    </head>
    <body>
      <h2>DB 로그 (최근 {limit}개)</h2>
      <p><a href="/status">Status로</a> | <a href="/">대시보드로</a></p>

      <div class="small">
        DB 파일: <code>{DB_PATH}</code><br/>
        필요하면 <code>DEFAULT_LOG_LIMIT</code> 또는 환경변수로 limit을 바꾸면 됨.
      </div>

      <table>
        <tr><th>ID</th><th>CAM_ID</th><th>object_id(=class)</th><th>시간</th></tr>
        <tbody id="rows"><tr><td colspan="4">불러오는 중...</td></tr></tbody>
      </table>

      <script>
        async function loadLogs() {{
          const res = await fetch("/api/logs?ts=" + Date.now());
          const data = await res.json();

          const tbody = document.getElementById("rows");
          tbody.innerHTML = "";

          if (data.error) {{
            tbody.innerHTML = `<tr><td colspan="4">${{data.error}}</td></tr>`;
            return;
          }}

          const items = data.items || [];
          if (items.length === 0) {{
            tbody.innerHTML = '<tr><td colspan="4">로그가 없습니다.</td></tr>';
            return;
          }}

          for (const it of items) {{
            const tr = document.createElement("tr");
            tr.innerHTML =
              `<td>${{it.id}}</td>` +
              `<td>${{it.cam_id}}</td>` +
              `<td>${{it.object_id}}</td>` +
              `<td>${{it.detected_at}}</td>`;
            tbody.appendChild(tr);
          }}
        }}
        loadLogs();
      </script>
    </body></html>
    """
    return make_response(html)

# =========================================================
# 7) main (여기서 스레드 시작)
# =========================================================

def main():
    # 웹캠 스트리밍 시작(기존 유지)
    Cam("web1", WEB1_DEVICE).start()
    Cam("web2", WEB2_DEVICE).start()

    # DB 테이블 준비(기존 DB 파일에 누적)
    if ENABLE_DB_LOG:
        db_init()

    # ROS(AMR 영상 + AMR class) 시작
    Ros().start()

    # Flask 실행
    # - use_reloader=False : flask가 2번 실행되는 문제 방지
    app.run(host="0.0.0.0", port=5000, debug=False, use_reloader=False, threaded=True)

if __name__ == "__main__":
    main()
