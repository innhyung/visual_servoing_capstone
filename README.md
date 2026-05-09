사용 컨트롤러: forward_position_controller

note -- servo_node가 내부적으로 Twist 명령을 미세한 Position 값으로 변환하여 송출함.

로봇 IP: 192.168.1.101 / 노트북 IP: 192.168.1.102

Terminal 1 - ur_control.launch.py 실행 후 티칭 펜던트에서 실행 버튼 클릭
    
    ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3 robot_ip:=192.168.1.101 reverse_ip:=192.168.1.102 launch_rviz:=false

Terminal 2 - ur_moveit.launch.py 실행 시 launch_servo:=true 옵션 넣고 실행
    
    ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3 launch_rviz:=true launch_servo:=true

Terminal 3 - forward_position_controller로 활성화
    
    ros2 control switch_controllers --deactivate forward_velocity_controller --activate forward_position_controller

Terminal 4 - rs_launch.py 실행 (align_depth 활성화 필수)
    
    ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true

Terminal 5 - switch_command_type 서비스를 호출하여 command_type: 1로 설정

    # TWIST mode
    ros2 service call /servo_node/switch_command_type moveit_msgs/srv/ServoCommandType "{command_type: 1}"

Terminal 6 - visual_servo_RS.py 실행
    
    python3 visual_servo_RS.py



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

### `visual_servo_RS.py` 안전성 개선
- color subscription을 `/image_raw/compressed` (CompressedImage)로 변경 → 대역폭/버퍼링 절감
- `search_radius`를 `search_radius_max`로 clip → IK 발산 방지
- publish 직전 NaN/Inf 검사 + `max_linear`로 속도 clip
- `warmup_frames`로 초기 N프레임 publish 건너뛰기 (servo 초기화 시간 확보)

### 새 파일
- `voice_target.py` — 음성 명령 → COCO 클래스 publish 노드 (위 섹션 참고)

### 알려진 이슈
- 로봇이 특이점(singularity) 자세에서 시작하면 MoveIt Servo가 NaN 출력 → `forward_position_controller`가 메시지 drop. 시작 전 정상 자세(예: shoulder_lift = -90°)로 이동시켜야 함.
- venv `(jazzy)` 프롬프트가 떠있어도 `which python3`이 시스템 Python을 가리킬 수 있음. venv Python을 명시적으로 호출하거나 `source /home/min/venv/jazzy/bin/activate` 사용.

