# RMD X-series 1 Mbps quick start

이 메모는 모터 전원 공급 장치를 구한 뒤 바로 통신 확인을 하기 위한 체크리스트입니다.

## 1. 준비물

- RMD X-series 모터
- 모터 정격에 맞는 DC 전원 공급 장치
- USB-CAN 어댑터 또는 CAN 인터페이스
- CANH/CANL 배선
- CAN 버스 종단저항 120 ohm
- 모터 설명서 기준 baud rate: 1 Mbps

전원을 넣기 전에는 모터가 갑자기 움직여도 위험하지 않게 축을 고정하거나 주변을 비워둡니다.

## 2. 패키지 설치

```bash
sudo apt-get install -y build-essential cmake
sudo apt-get install -y can-utils iproute2 linux-modules-extra-$(uname -r)
sudo apt-get install -y python3 python3-pip python3-pybind11
```

Python 바인딩을 설치합니다.

```bash
pip3 install .
```

## 3. CAN 인터페이스를 1 Mbps로 올리기

USB-CAN 장치를 꽂은 뒤 인터페이스 이름을 확인합니다.

```bash
ip link show
```

이 프로젝트에서는 `can_arm`으로 사용합니다. 이미 올라가 있으면 한 번 내린 뒤 1 Mbps로 다시 올립니다.

```bash
sudo ip link set can_arm down
sudo ip link set can_arm up type can bitrate 1000000
ip -details link show can_arm
```

수신 모니터링은 별도 터미널에서 켜둡니다.

```bash
candump can_arm
```

## 4. 첫 통신 확인

기본 모터 ID가 `1`이면 CAN 주소는 `0x140 + 1 = 0x141`입니다.

먼저 모터를 움직이지 않는 smoke test를 실행합니다.

```bash
python3 scripts/rmd_smoke_test.py --interface can_arm --id 1
```

성공하면 firmware version, status, angle 값이 출력됩니다.

## 5. 첫 구동

통신이 확인된 뒤에만 낮은 속도로 작은 위치 명령을 보냅니다.

```bash
python3 scripts/rmd_smoke_test.py --interface can_arm --id 1 --move-position 10 --max-speed 30 --yes-move
```

원점 쪽으로 되돌릴 때:

```bash
python3 scripts/rmd_smoke_test.py --interface can_arm --id 1 --move-position 0 --max-speed 30 --yes-move
```

테스트 후 모터를 끄려면:

```bash
python3 scripts/rmd_smoke_test.py --interface can_arm --id 1 --shutdown
```

## 6. 문제 확인

- `can_arm`이 없으면 USB-CAN 드라이버/펌웨어와 인터페이스 이름 설정을 확인합니다.
- 응답이 없으면 baud rate가 1 Mbps인지 확인합니다.
- 모터 ID가 다르면 `--id` 값을 바꿉니다.
- `candump can_arm`에 에러 프레임만 보이면 CANH/CANL, GND 기준, 종단저항, 전원 상태를 확인합니다.
- 모터가 진동만 하고 움직이지 않으면 이미 목표 위치 근처일 수 있습니다.
