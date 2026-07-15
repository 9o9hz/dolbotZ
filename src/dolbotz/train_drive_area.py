#!/usr/bin/env python3
"""
dolbotZ drivable-area segmentation 학습 스크립트

환경:
  - 학습: GPU 노트북 (CUDA)
  - 배포: GPU 없는 i7 온보드 PC -> OpenVINO로 변환해서 사용

사용법:
  pip install ultralytics roboflow
  export ROBOFLOW_API_KEY=<Roboflow 대시보드(Settings -> API Keys)에서 발급한 키>
  python train_drive_area.py

끝나면 생성되는 것:
  runs/segment/dolbotz_seg_v1/weights/best.pt          <- GPU/일반 PyTorch용
  runs/segment/dolbotz_seg_v1/weights/best_openvino_model/  <- i7 배포용 (이 폴더를 통째로 복사)

yolo26n-seg.pt는 git에 포함되지 않음(.gitignore). YOLO("yolo26n-seg.pt") 최초
실행 시 ultralytics가 자동 다운로드하며, 오프라인 환경(대회장 등)에서 재학습이
필요할 수 있다면 사전에 파일을 받아 리포 루트에 두어야 함.
"""

import os
import shutil
from pathlib import Path

from roboflow import Roboflow
from ultralytics import YOLO

# ---------------------------------------------------------------------------
# 0. 설정
# ---------------------------------------------------------------------------
RUN_NAME = "dolbotz_seg_v1"   # 재학습 시 v2, v3...로 바꿔서 결과 폴더 구분
EPOCHS = 150
IMGSZ = 640
BATCH = 16                    # GPU VRAM 8GB 미만이면 8로 낮추세요
PATIENCE = 40                 # val 성능 40 epoch 개선 없으면 자동 조기 종료

# ---------------------------------------------------------------------------
# 1. 데이터셋 다운로드 (이미 받아뒀으면 자동 스킵됨)
# ---------------------------------------------------------------------------
# API 키는 환경변수 ROBOFLOW_API_KEY로만 전달한다 — 절대 코드에 하드코딩하지
# 말 것 (이 리포는 공개 저장소이며, 과거에 실제로 키가 평문으로 커밋된 적이
# 있었다. 코드에서 지운다고 git 히스토리에서 사라지지 않으므로, 과거에
# 노출됐던 키는 Roboflow 대시보드에서 반드시 revoke하고 새로 발급받을 것).
api_key = os.environ.get("ROBOFLOW_API_KEY")
if not api_key:
    raise SystemExit(
        "환경변수 ROBOFLOW_API_KEY가 설정되지 않았습니다.\n"
        "  export ROBOFLOW_API_KEY=<Roboflow 대시보드에서 발급한 키>\n"
        "실행 후 다시 시도하세요."
    )
rf = Roboflow(api_key=api_key)
project = rf.workspace("s-workspace-a8lvp").project("drive-area-slvqh-1gauq")
version = project.version(1)
dataset = version.download("yolo26")
data_yaml = str(Path(dataset.location) / "data.yaml")
print(f"[1/4] dataset ready: {data_yaml}")

# ---------------------------------------------------------------------------
# 2. 학습 (GPU)
# ---------------------------------------------------------------------------
model = YOLO("yolo26n-seg.pt")  # 사전학습 가중치에서 fine-tuning

results = model.train(
    data=data_yaml,
    epochs=EPOCHS,
    imgsz=IMGSZ,
    batch=BATCH,
    device=0,                # 첫 번째 GPU
    patience=PATIENCE,
    name=RUN_NAME,
    # --- augmentation (Roboflow에서 이미 켰다면 아래는 보수적으로) ---
    degrees=10.0,            # 회전 ±10°
    fliplr=0.5,              # 수평 플립 50%
    flipud=0.0,              # 수직 플립 금지 (카메라가 뒤집힐 일 없음)
    hsv_h=0.01,              # 색조는 최소한만 (트랙 청록색이 핵심 특징)
    hsv_s=0.4,               # 채도 변화 -> 눈/탈색 상황 대비
    hsv_v=0.4,               # 밝기 변화 -> 대회장 조명 대비
    mosaic=0.0,              # 바닥 영역 학습엔 모자이크 비활성
    erasing=0.4,             # random erasing -> Cutout 효과 (winter/가림 대비)
    # ------------------------------------------------------------
    plots=True,
)
print(f"[2/4] training done: {results.save_dir}")

best_pt = Path(results.save_dir) / "weights" / "best.pt"

# ---------------------------------------------------------------------------
# 3. 검증 지표 확인
# ---------------------------------------------------------------------------
best_model = YOLO(str(best_pt))
metrics = best_model.val(data=data_yaml, device=0)
print("[3/4] validation metrics")
print(f"  mask mAP50    : {metrics.seg.map50:.4f}")
print(f"  mask mAP50-95 : {metrics.seg.map:.4f}")
if metrics.seg.map50 < 0.85:
    print("  !! mask mAP50이 0.85 미만입니다. 라벨 품질/데이터 구성을 점검하세요.")

# ---------------------------------------------------------------------------
# 4. i7 배포용 OpenVINO 변환 (CPU 추론 가속)
# ---------------------------------------------------------------------------
export_path = best_model.export(format="openvino", imgsz=IMGSZ, half=False)
print(f"[4/4] OpenVINO export: {export_path}")
print()
print("=" * 60)
print("다음 단계:")
print(f"  1. '{export_path}' 폴더를 통째로 i7 PC로 복사")
print("  2. i7에서:  pip install ultralytics openvino")
print("  3. 코드에서:")
print(f'       model = YOLO("{Path(export_path).name}")')
print("       result = model.predict(frame)   # CPU에서 자동으로 OpenVINO 사용")
print("=" * 60)