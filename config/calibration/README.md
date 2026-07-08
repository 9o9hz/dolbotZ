# config/calibration/

카메라별 실측 캘리브레이션 결과(intrinsic/extrinsic, 마운트 각도 등)를 저장하는
피클 파일이 들어가는 자리. `dolbotz.utils.paths.load_calibration(serial_no)`가
이 디렉토리를 읽는다.

**아직 이 피클을 생성하는 캘리브레이션 스크립트는 없다.** 이번 작업 범위가
아니며, 나중에 별도로 작성할 예정이다. 지금은 파일명/스키마 컨벤션만
정해둔다.

## 파일명 컨벤션

```
{camera_model}_{serial_no}.pkl
```

예:
- `D435I_339222071362.pkl` — 주행용 카메라
- `D455_117222251401.pkl` — 로봇팔용 카메라

`load_calibration()`은 `serial_no`만으로 `*_{serial_no}.pkl` 패턴 글롭 탐색을
하므로 호출부는 `camera_model` 접두사를 몰라도 된다. 단, 피클 내부의
`serial_no` 필드가 파일명이 가리키는 값과 다르면 `load_calibration()`이
`ValueError`를 던진다(잘못 복사/이름 변경된 파일을 조용히 쓰는 사고 방지).

## 피클 스키마

피클 내용은 아래 키를 가진 `dict`:

| 키 | 타입 | 설명 |
|---|---|---|
| `serial_no` | `str` | 카메라 시리얼 번호. 파일명과 일치해야 함 |
| `camera_model` | `str` | 예: `"D435I"`, `"D455"` |
| `measured_at` | `str` | ISO8601 문자열 (예: `"2026-07-08T00:00:00"`) |
| `camera_height_m` | `float` | 지면 위 카메라 높이 [m] |
| `camera_pitch_offset_deg` | `float` | 고정 카메라 장착 피치 (기수 하향이 양수) [deg] |
| `camera_roll_offset_deg` | `float` | 고정 카메라 장착 롤 [deg] |
| `camera_matrix` | `np.ndarray (3x3)` 또는 `None` | 핀홀 내부 파라미터. 없으면 CameraInfo 토픽 값 사용 |
| `dist_coeffs` | `np.ndarray` 또는 `None` | 왜곡 계수. 없으면 CameraInfo 토픽 값 사용 |
| `accel_reference_body` | `np.ndarray (3,)` 또는 `None` | 평지에서 실측한 기준 가속도(body 프레임) |
| `complementary_filter_alpha` | `float` (선택) | 상보필터 alpha 오버라이드. 키 자체가 없어도 됨 |

`camera_matrix`/`dist_coeffs`/`accel_reference_body`가 `None`인 필드는 해당
값을 이 피클로 오버라이드하지 않고 기존 소스(CameraInfo 토픽, ROS 파라미터
기본값 등)를 그대로 쓴다는 뜻이다.

## 호출부(ROS 노드)의 폴백 규칙

`elevation_map_node`/`flat_drive_node`는 시작 시 `camera_serial_no` 파라미터
(기본값 빈 문자열)로 `dolbotz.utils.attitude.resolve_mount_defaults(serial_no)`를
호출해 `camera_height_m`/`camera_pitch_offset_deg`/`camera_roll_offset_deg`/
`complementary_filter_alpha`의 `declare_parameter` 기본값을 정한다. 우선순위는:

1. `--ros-args -p camera_height_m:=X` 등으로 명시적으로 넘긴 값 (항상 최우선)
2. 이 디렉토리의 캘리브레이션 피클에 해당 키가 있으면 그 값
3. 둘 다 없으면 `dolbotz/utils/attitude.py`의 `MOUNT_*_PLACEHOLDER` /
   `COMPLEMENTARY_FILTER_ALPHA_PLACEHOLDER` 상수

지금은 피클이 하나도 없으므로 모든 노드가 항상 3번(코드 상수)을 쓴다 —
나중에 캘리브레이션 스크립트가 피클을 만들면 코드 변경 없이 자동으로
2번이 반영된다.
