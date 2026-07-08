"""
리포/패키지 경로 해석 공용 유틸리티 — 실행 위치(cwd)나 사용자 홈 경로에
관계없이 config/, config/models/, config/calibration/ 디렉토리를 안정적으로
찾는다.

배경: arm_pickup.py의 model_path 기본값이 과거 절대경로로 하드코딩되어 있다가
(`/home/j/dolbotZ/...` -> `/home/jecs/dolbotZ/...`, 커밋 6056cf8) 사용자/머신이
바뀌면서 다시 깨진 전례가 있다. 이 모듈은 그 문제를 근본적으로 해결한다.

두 가지 실행 환경을 모두 지원하는 하이브리드 방식을 쓴다:

  1. colcon install 환경 (심볼릭이든 일반 설치든): ament_index_python으로
     'dolbotz' 패키지의 share 디렉토리(예: install/dolbotz/share/dolbotz)를
     정확히 찾는다. config/는 setup.py의 data_files로 이 share 디렉토리
     아래 설치되므로, 실제 로봇 배포(정식 colcon install) 환경에서 항상
     안전하게 동작하는 유일한 방법이다 — 일반(비심볼릭) install에서는
     설치된 .py 파일이 site-packages 트리에, config/는 share 트리에 있어
     서로 부모-자식 관계가 아니므로, __file__ 기준 상위 탐색으로는 원리적으로
     찾을 수 없다.
  2. ROS 빌드 없이 소스에서 바로 실행하는 개발/테스트 환경(PYTHONPATH=src):
     ament_index가 'dolbotz'를 찾지 못하면(ImportError 또는
     PackageNotFoundError), 이 파일(__file__) 위치에서 위로 올라가며
     config/와 src/dolbotz/가 모두 있는 지점을 리포 루트로 판정한다.
     (colcon --symlink-install 환경에서도 설치된 .py가 소스로 심볼릭
     연결되어 있으므로 이 폴백이 마침 같이 동작한다.)

두 방법이 모두 실패하면, 무엇을 시도했는지 전부 포함한 에러를 낸다.
"""

from __future__ import annotations

import pickle
from pathlib import Path


def get_package_share_dir() -> Path:
    """dolbotz 패키지의 공유 데이터 루트. 모듈 docstring의 우선순위를 따른다."""
    tried: list[str] = []

    try:
        from ament_index_python.packages import (
            PackageNotFoundError,
            get_package_share_directory,
        )
    except ImportError as exc:
        tried.append(f'ament_index_python import 실패: {exc}')
    else:
        try:
            return Path(get_package_share_directory('dolbotz'))
        except PackageNotFoundError as exc:
            tried.append(f"ament_index_python.get_package_share_directory('dolbotz') 실패: {exc}")

    try:
        return get_repo_root()
    except RuntimeError as exc:
        tried.append(str(exc))

    raise RuntimeError(
        'dolbotz 패키지의 공유 디렉토리를 찾지 못했습니다. 시도한 방법:\n  - '
        + '\n  - '.join(tried)
    )


def get_repo_root() -> Path:
    """소스 트리 기준 리포 루트를 __file__ 위치부터 상위로 올라가며 찾는다.

    config/와 src/dolbotz가 모두 존재하는 첫 조상 디렉토리를 리포 루트로
    판정한다. 일반(비심볼릭) colcon install 환경에서는 config/가 아예 다른
    트리(share/)에 있으므로 이 함수만으로는 못 찾는다 — 그 경우엔
    get_package_share_dir()를 쓸 것 (내부적으로 이 함수를 폴백으로 사용함).
    """
    here = Path(__file__).resolve()
    for candidate in here.parents:
        if (candidate / 'config').is_dir() and (candidate / 'src' / 'dolbotz').is_dir():
            return candidate
    raise RuntimeError(
        '__file__ 상위 탐색으로 리포 루트를 찾지 못했습니다 (config/, src/dolbotz/가 '
        f'모두 있는 상위 디렉토리 없음). 시도한 경로: {[str(p) for p in here.parents]}'
    )


def get_config_dir() -> Path:
    return get_package_share_dir() / 'config'


def get_models_dir() -> Path:
    return get_config_dir() / 'models'


def get_calibration_dir() -> Path:
    return get_config_dir() / 'calibration'


def load_calibration(serial_no: str) -> dict | None:
    """config/calibration/{camera_model}_{serial_no}.pkl이 있으면 로드해서
    dict로 반환한다. camera_model 접두사는 몰라도 되도록 serial_no만으로
    글롭 탐색한다 (`config/calibration/README.md`의 네이밍 컨벤션 참고).

    파일이 없으면 None을 반환한다 — 호출부(ROS 노드)는 이 경우 기존 ROS
    파라미터 기본값으로 폴백해야 한다. 지금은 이 피클을 생성하는 캘리브레이션
    스크립트가 아직 없으므로 항상 None이 반환되는 것이 정상이다.
    """
    if not serial_no:
        return None

    calibration_dir = get_calibration_dir()
    if not calibration_dir.is_dir():
        return None

    matches = sorted(calibration_dir.glob(f'*_{serial_no}.pkl'))
    if not matches:
        return None

    with open(matches[0], 'rb') as f:
        data = pickle.load(f)

    if data.get('serial_no') != serial_no:
        raise ValueError(
            f"{matches[0].name}의 내부 serial_no({data.get('serial_no')!r})가 파일명이 "
            f'가리키는 serial_no({serial_no!r})와 일치하지 않습니다.'
        )
    return data
