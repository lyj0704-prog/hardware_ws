# Jetson Serial Quickstart (1-page)

이 문서는 "다른 사람이 Jetson에서 ESP32와 시리얼 통신을 바로 시작"하기 위한 최소 절차입니다.

## 1. 준비물

- Jetson (Ubuntu + ROS 2 Humble 환경)
- ESP32 보드
- USB-UART 또는 Jetson UART 핀
- 3.3V 레벨 TX/RX/GND 배선

주의:
- TX/RX는 교차 연결 (Jetson TX -> ESP32 RX, Jetson RX -> ESP32 TX)
- GND는 반드시 공통 연결
- 5V TTL 직접 연결 금지

## 2. 코드 받기

```bash
cd ~
git clone <YOUR_REPO_URL> hardware_ws
cd hardware_ws
```

## 3. ROS 의존성 설치

ROS 2 Humble이 이미 설치되어 있다는 전제입니다.

```bash
source /opt/ros/humble/setup.bash
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

`ackermann_msgs`가 누락되면:

```bash
sudo apt install ros-humble-ackermann-msgs
```

## 4. 빌드

```bash
cd ~/hardware_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select motor_serial_bridge_cpp
source install/setup.bash
```

## 5. ESP32 펌웨어 업로드

아래 파일을 ESP32에 업로드:
- `esp32/esp32_motor_servo_bridge/esp32_motor_servo_bridge.ino`

Jetson 브리지와 ESP32 펌웨어의 baud rate를 동일하게 맞추세요 (기본 115200).

## 6. 시리얼 포트 설정

기본 설정 파일:
- `src/motor_serial_bridge_cpp/config/motor_serial_bridge.yaml`

대표 포트 예시:
- Jetson UART: `/dev/ttyTHS1` 또는 `/dev/ttyAMA0`
- USB UART: `/dev/ttyUSB0`

포트 확인:

```bash
ls /dev/ttyTHS* /dev/ttyAMA* /dev/ttyUSB* 2>/dev/null
```

권한 문제(`Permission denied`)가 나면:

```bash
sudo usermod -a -G dialout $USER
newgrp dialout
```

## 7. 실행

```bash
cd ~/hardware_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch motor_serial_bridge_cpp motor_serial_bridge.launch.py
```

커스텀 설정 파일 사용:

```bash
ros2 launch motor_serial_bridge_cpp motor_serial_bridge.launch.py \
  config:=/absolute/path/to/motor_serial_bridge.yaml
```

## 8. 빠른 점검

토픽 확인:

```bash
ros2 topic list
```

브리지 출력 확인:

```bash
ros2 topic echo /diagnostics
ros2 topic echo /odom
```

명령 테스트(예시):

```bash
ros2 topic pub /ackermann_cmd ackermann_msgs/msg/AckermannDriveStamped \
"{drive: {speed: 0.5, steering_angle: 0.1}}" -1
```

## 9. 자주 막히는 포인트

- 포트 이름 불일치: YAML의 `port`와 실제 장치명 확인
- baud mismatch: Jetson/ESP32 양쪽 동일 값 설정
- 배선 오류: TX/RX 교차, GND 공통 여부 확인
- 권한 문제: `dialout` 그룹 추가 후 셸 재시작
- 펌웨어 미업로드: ESP32가 `CMD ...` / `FB ...` 프로토콜을 처리해야 함
