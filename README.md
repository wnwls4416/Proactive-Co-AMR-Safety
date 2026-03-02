# Proactive-Co-AMR-Safety
SLAM(위치추정 및 공간지도 생성) 기반 자율주행 로봇 시스템 프로젝트

# 🛡️ Proactive Co-AMR Safety System
> **AMR 협업형 실내 안전위협요소 선제 대응 시스템**

이 프로젝트는 실내 공간에서 발생할 수 있는 안전 위협 요소(장애물, 균열, 액체 등)를 고정형 웹캠으로 선제적으로 감지하고, 다중 AMR(자율주행 로봇)이 협업하여 정밀 탐색 및 후속 조치를 수행하는 통합 관제 시스템입니다.

## ✨ 핵심 기능 (Key Features)
* **사각지대 없는 선제 감지**: 2대의 웹캠이 공간을 4x4 격자(16개 섹션)로 분할하여 모니터링하며, YOLO를 이용해 위협 객체를 실시간으로 식별합니다.
* **다중 로봇 협업 (Multi-AMR Collaboration)**:
  * **AMR 1 (1차 출동 및 정밀 측정)**: 웹캠이 알려준 대략적인 섹션으로 이동 후, OAK-D 3D 카메라를 통해 객체의 정확한 맵 좌표(x, y)를 추출합니다.
  * **AMR 2 (2차 출동 및 조치)**: AMR 1이 전송한 정밀 좌표를 받아 출동하여 임무를 수행하고 복귀합니다.
* **실시간 웹 통합 관제 (Web Dashboard)**: Flask 기반의 웹 대시보드에서 4개의 카메라(웹캠 2, AMR 2) 뷰를 동시에 확인하고, DB에 저장된 감지 이력을 관리합니다. 영상 클릭 시 확대되는 모달 기능을 지원합니다.

---

## 🏗️ 시스템 워크플로우 (System Workflow)

본 시스템은 총 6단계(Stage 1 ~ 6)에 걸쳐 작동합니다.

1. **위협 감지 및 구역 할당**: 웹캠이 위협 객체를 감지하고, 해당 객체가 위치한 섹션 번호(1~16)와 클래스 ID를 ROS2 토픽으로 발행합니다.
2. **Stage 1 & 2 (1차 접근)**: 대기 중이던 첫 번째 로봇이 신호를 받아 언도킹(Undock) 후, 해당 섹션의 안전 좌표로 Nav2 주행을 시작합니다.
3. **Stage 3 & 4 (정밀 위치 추정 및 근접)**: 도착한 첫 번째 로봇이 OAK-D 카메라와 YOLO를 활용해 객체를 화면 중앙에 정렬합니다. Depth 데이터를 통해 거리를 측정하고, `tf2`를 이용해 객체의 절대 맵 좌표를 계산하여 두 번째 로봇에게 전송합니다.
4. **Stage 5 & 6 (협업 및 최종 복귀)**: 두 번째 로봇이 수신된 정밀 좌표로 출동합니다. 임무(대기 및 후속 조치) 완료 후 복귀 신호를 발행하며, 두 로봇 모두 초기 위치로 돌아가 도킹(Docking)을 완료합니다.

---

## 📂 파일 구조 (Repository Structure)

| 파일명 | 설명 |
|---|---|
| `pc3_stage1_2_node.py` | 웹캠 신호를 수신하여 첫 번째 AMR을 해당 구역으로 이동시키는 ROS2 노드 |
| `pc3_stage3_4_node.py` | 객체 정밀 탐색, TF 좌표 변환, 두 번째 AMR로 좌표를 전송하는 ROS2 노드 |
| `pc3_stage5_6_node.py` | 전달받은 맵 좌표로 두 번째 AMR(TurtleBot4)이 이동하여 임무 수행 후 복귀하는 ROS2 노드 |
| `appdb_merged_kr_fix.py` | 영상 캡처, YOLO 인식, DB 로깅, 통신을 통합 관리하는 Flask 메인 서버 |
| `dashboard_v8b_modal.html` | 실시간 4채널 영상 스트리밍 및 DB 이력을 보여주는 웹 대시보드 프론트엔드 |
| `section_utils.py` | 웹캠 화면 내 객체의 픽셀 좌표를 4x4 격자 섹션 번호로 변환해 주는 유틸리티 |

---

## 💻 요구 사항 (Prerequisites)

* **OS**: Ubuntu 22.04
* **ROS**: ROS2 Humble
* **Hardware**: TurtleBot4 & iRobot Create 3 Base, OAK-D Camera, USB Webcams x 2
* **Python Packages**:
  ```bash
  pip install rclpy flask opencv-python numpy ultralytics cv_bridge tf2_ros
  ```

---

## 🚀 실행 방법 (How to Run)

### 1. 작업 공간 빌드 및 환경 설정
터미널을 열고 워크스페이스를 빌드한 후 환경 변수를 설정합니다.

```bash
cd ~/rokey_ws
colcon build
source install/setup.bash
export ROS_DOMAIN_ID=5
```

### 2. 웹 관제 대시보드 서버 실행
Flask 기반의 메인 서버와 YOLO 모니터링을 시작합니다. (경로는 본인 환경에 맞게 수정하세요)

```bash
export YOLO_AMR_MODEL_PATH=/home/rokey/rokey_ws/src/pc3/models/best.pt
export HTML_VERSION=modal
python3 appdb_merged_kr_fix.py
```
> 👉 서버 실행 후 브라우저에서 `http://localhost:5000` 에 접속하여 대시보드를 확인합니다.

### 3. 로봇 협업 노드 순차적 실행
각각 **새로운 터미널**을 열고 환경 변수(`source install/setup.bash`, `export ROS_DOMAIN_ID=5`)를 설정한 뒤, 다음 노드들을 순서대로 실행합니다.

**터미널 1 (첫 번째 로봇 출동):**
```bash
python3 pc3_stage1_2_node.py
```

**터미널 2 (첫 번째 로봇 정밀 탐색 및 좌표 전송):**
```bash
python3 pc3_stage3_4_node.py
```

**터미널 3 (두 번째 로봇 출동 및 조치):**
```bash
python3 pc3_stage5_6_node.py
```
