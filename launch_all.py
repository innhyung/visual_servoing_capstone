#!/usr/bin/env python3
"""
UR3e 비주얼 서보잉 통합 런처
터미널 6개를 하나로 합친 런치 파일

실행:
    ros2 launch ~/ros2_ws_cap/src/visual_servoing_capstone/launch_all.py

타이밍 조정이 필요하면 각 TimerAction의 period 값을 수정하세요.
"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

ROBOT_IP   = '192.168.1.101'
REVERSE_IP = '192.168.1.102'
UR_TYPE    = 'ur3e'
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
WS_ROOT    = os.path.expanduser('~/ros2_ws_cap')
VENV_PY    = '/home/min/venv/jazzy/bin/python3'


def generate_launch_description():

    # ── Terminal 1: UR 드라이버 (pre-flight 체크 + 자동 Play 포함) ──────────────
    # start_ur3.sh가 좀비 정리 → ros2 launch → 0.6s 후 dashboard play 자동 전송
    ur_driver = ExecuteProcess(
        cmd=['bash', os.path.join(WS_ROOT, 'start_ur3.sh')],
        output='screen',
    )

    # ── Terminal 4: RealSense D455 (드라이버와 병렬 시작) ───────────────────────
    # align_depth 필수
    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('realsense2_camera'),
            '/launch/rs_launch.py'
        ]),
        launch_arguments={'align_depth.enable': 'true'}.items()
    )

    # ── Terminal 2: MoveIt + servo_node (드라이버 연결 대기 후 ~5s) ────────────
    moveit = TimerAction(period=5.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('ur_moveit_config'),
                '/launch/ur_moveit.launch.py'
            ]),
            launch_arguments={
                'ur_type':      UR_TYPE,
                'launch_rviz':  'true',
                'launch_servo': 'true',
            }.items()
        )
    ])

    # ── Terminal 3: forward_position_controller 활성화 (~12s) ──────────────────
    # MoveIt 완전 로드 후 실행
    activate_ctrl = TimerAction(period=12.0, actions=[
        ExecuteProcess(
            cmd=['ros2', 'control', 'switch_controllers',
                 '--activate', 'forward_position_controller'],
            output='screen',
        )
    ])

    # ── Terminal 5: servo TWIST 모드 전환 (~14s) ───────────────────────────────
    set_twist = TimerAction(period=14.0, actions=[
        ExecuteProcess(
            cmd=['ros2', 'service', 'call',
                 '/servo_node/switch_command_type',
                 'moveit_msgs/srv/ServoCommandType',
                 '{command_type: 1}'],
            output='screen',
        )
    ])

    # ── Terminal 6: visual_servo_RS.py (~17s) ──────────────────────────────────
    # venv Python 사용 (ultralytics, cv2 등 패키지 위치)
    visual_servo = TimerAction(period=17.0, actions=[
        ExecuteProcess(
            cmd=[VENV_PY, os.path.join(SCRIPT_DIR, 'visual_servo_RS.py')],
            output='screen',
            cwd=SCRIPT_DIR,
        )
    ])

    return LaunchDescription([
        ur_driver,
        realsense,
        moveit,
        activate_ctrl,
        set_twist,
        visual_servo,
    ])
