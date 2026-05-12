사용 컨트롤러: forward_position_controller

note -- servo_node가 내부적으로 Twist 명령을 미세한 Position 값으로 변환하여 송출함.

로봇 IP: 192.168.1.101 / 노트북 IP: 192.168.1.102

---

## 빠른 실행 (통합 런치)

터미널 6개를 하나로 합친 런치 파일:

    ros2 launch ~/ros2_ws_cap/src/visual_servoing_capstone/launch_all.py

자동 실행 순서: UR 드라이버(+Play) → RealSense → MoveIt+Servo → 컨트롤러 활성화 → TWIST 모드 → visual_servo_RS.py

> 타이밍 문제 시 `launch_all.py` 안의 `TimerAction period` 값을 조정하세요.

---

## 수동 실행 순서 (단계별)

Terminal 1 - start_ur3.sh 실행 (pre-flight 체크 + 드라이버 + 자동 Play)

    bash ~/ros2_ws_cap/start_ur3.sh

    # 스크립트가 자동으로:
    #   - ping / IP 확인
    #   - 좀비 UR 프로세스 정리
    #   - ros2 launch ur_robot_driver ... ur_type:=ur3e 실행
    #   - 0.6초 후 티칭 펜던트에 Play 명령 자동 전송
    # Remote Control 모드가 아닌 경우: 터미널에 "controller_manager: 500 Hz" 뜨면 즉시 ▶ Play

Terminal 2 - ur_moveit.launch.py 실행 시 launch_servo:=true 옵션 넣고 실행

    ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3e launch_rviz:=true launch_servo:=true

Terminal 3 - forward_position_controller 활성화 (매번 실행 필요)

    ros2 control switch_controllers --activate forward_position_controller

    # 확인:
    ros2 control list_controllers | grep forward_position
    # → forward_position_controller ... active 이어야 함

Terminal 4 - rs_launch.py 실행 (align_depth 활성화 필수)

    ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true

Terminal 5 - switch_command_type 서비스를 호출하여 command_type: 1로 설정

    # TWIST mode
    ros2 service call /servo_node/switch_command_type moveit_msgs/srv/ServoCommandType "{command_type: 1}"

Terminal 6 - visual_servo_RS.py 실행

    python3 visual_servo_RS.py

---

## 파이프라인 진단 명령

    ros2 topic hz /servo_node/delta_twist_cmds          # 코드→servo 명령 확인 (~17 Hz)
    ros2 topic echo /servo_node/status                   # code:0 = 정상
    ros2 topic hz /forward_position_controller/commands  # servo→controller 확인 (~250 Hz)
    ros2 control list_controllers | grep forward_position # active 확인



-----------------------------------------------------------

## voice_target.py — 음성 명령으로 타겟 객체 publish

음성으로 "리모컨 찾아줘" 같이 말하면, **한국어 → COCO 클래스명**으로 변환해서 `/target_object` 토픽에 publish하는 ROS2 노드.

### 실행 방법

venv 활성화 + ROS2 환경 source된 셸에서:

```bash
python3 voice_target.py
```

또는 venv Python 직접 호출:

```bash
/home/min/venv/jazzy/bin/python3 voice_target.py
```

### 동작 흐름

1. 시작 → 주변 소음 1초 측정 → 🔊 **"어떤 물건을 찾고 싶으신가요?"**
2. 마이크 listen → Google STT (한국어)로 텍스트 변환
3. 매핑되는 한국어 키워드 발견 시:
   - 🔊 **"네 리모컨을 찾을게요"** (조사 자동 처리)
   - `/target_object` 토픽에 `"remote"` (영어 COCO 클래스명) publish
4. **"그만"** / "종료" / "끝" 인식 시 → 🔊 **"네 물건찾기를 종료할게요"** → 노드 종료

### 토픽 확인

```bash
ros2 topic echo /target_object
# data: 'remote'
```

### 의존성

- `speech_recognition` (Google STT)
- `gTTS` (Google 한국어 TTS, 무료)
- `ffplay` (mp3 재생, 시스템에 이미 있음)
- `pyaudio` (마이크 입력)

설치는 `pip install -r requirement.txt` + `pip install gTTS`.

### 매핑된 객체 (KO_TO_COCO dict)

리모컨, 마우스, 키보드, 노트북, 휴대폰/핸드폰/스마트폰, 책, 컵, 병/물병, 가방/백팩, 핸드백, 우산, 시계, 가위, 칫솔, 사과, 바나나, 오렌지, 케이크, 도넛, 피자, 샌드위치, 포크, 칼, 숟가락, 그릇, 와인잔, TV/텔레비전, 의자, 소파/쇼파, 침대, 식탁/테이블, 변기, 화병/꽃병, 사람, 고양이, 강아지/개

추가하려면 `voice_target.py`의 `KO_TO_COCO` dict에 한 줄 추가.

---

## 최근 변경 사항

### `visual_servo_RS.py` 나선형 탐색 수식 수정
- **버그**: `vx = R·cos(θ)` — 위치값을 속도로 잘못 사용 → ±13mm 진동만 반복
- **수정**: `vx = -R·ω·sin(θ)`, `vy = R·ω·cos(θ)` (원 궤적의 속도 미분값)
- ω = 1.5 rad/s, 최대속도 0.06 m/s (max_linear 0.08 이내)
- 타겟 상실(SERVOING→SEARCHING) 시 search_angle, search_radius 초기화 추가

### `visual_servo_RS.py` 안전성 개선
- color subscription을 `/image_raw/compressed` (CompressedImage)로 변경 → 대역폭/버퍼링 절감
- `search_radius`를 `search_radius_max`로 clip → IK 발산 방지
- publish 직전 NaN/Inf 검사 + `max_linear`로 속도 clip
- `warmup_frames`로 초기 N프레임 publish 건너뛰기 (servo 초기화 시간 확보)

### `start_ur3.sh` 자동 Play 추가
- 드라이버 시작 0.6초 후 dashboard(포트 29999)로 Play 명령 자동 전송
- 이유: hardware interface configuration timeout이 1초라 수동 Play가 불가능했음

### 새 파일
- `voice_target.py` — 음성 명령 → COCO 클래스 publish 노드 (위 섹션 참고)

### 알려진 이슈
- 로봇이 특이점(singularity) 자세에서 시작하면 MoveIt Servo가 NaN 출력 → `forward_position_controller`가 메시지 drop. 시작 전 정상 자세(예: shoulder_lift = -90°)로 이동시켜야 함.
- `forward_position_controller`는 매번 수동으로 activate 해야 함 (Terminal 3).
- venv `(jazzy)` 프롬프트가 떠있어도 `which python3`이 시스템 Python을 가리킬 수 있음. venv Python을 명시적으로 호출하거나 `source /home/min/venv/jazzy/bin/activate` 사용.

