# config/models/

리포 전체에서 쓰는 학습된 모델 가중치를 한곳에 모아둔다. 예전에는 리포
루트와 `runs/segment/`에 흩어져 있었고, `arm_pickup.py`의 `model_path`
기본값이 절대경로로 하드코딩되어 있다가 사용자/머신이 바뀌면서 깨진 전례가
있다(`/home/j/dolbotZ/...` -> `/home/jecs/dolbotZ/...`, 커밋 `6056cf8`).
이제 모든 노드가 `dolbotz.utils.paths.get_models_dir()`로 이 디렉토리를
실행 환경과 무관하게 찾는다.

## 이동 내역

| 파일/디렉토리 | 원래 위치 | 사용 노드 |
|---|---|---|
| `supplybest.pt` | 리포 루트 | `arm_pickup_node` |
| `dolbotz_seg_v1/` | `runs/segment/dolbotz_seg_v1/weights/` | `flat_drive_node` (`best_openvino_model/` 사용) |

`dolbotz_seg_v1/best.pt`는 GPU/일반 PyTorch용 원본 가중치, `best_openvino_model/`은
CPU 추론용 OpenVINO IR 변환본(`best.bin`, `best.xml`, `metadata.yaml`)이다.
`flat_drive_node`는 `best_openvino_model/`만 사용한다.

이동 사유: 경로 일관성(모델 절대경로가 실행 환경에 따라 깨지는 문제 근절)과
모델 파일 한곳 관리. `git mv`로 옮겨서 커밋 이력은 보존된다
(`git log --follow -- config/models/...`로 원래 커밋까지 추적 가능).

## 새로 학습한 모델을 반영하려면

`train_drive_area.py`는 여전히 `runs/segment/{RUN_NAME}/weights/`에 결과를
남긴다(학습 산출물을 검증 전에 곧바로 배포 위치에 덮어쓰지 않기 위한 의도적
분리 — howtorun.md 참고). 검증 후 배포하려면 수동으로 이 디렉토리에
복사한다:

```bash
cp runs/segment/{RUN_NAME}/weights/best.pt config/models/dolbotz_seg_v1/
cp -r runs/segment/{RUN_NAME}/weights/best_openvino_model config/models/dolbotz_seg_v1/
```
