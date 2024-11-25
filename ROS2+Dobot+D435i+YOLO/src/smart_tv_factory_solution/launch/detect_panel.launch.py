from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    # Dobot bringup launch
    dobot_bringup_launch = ExecuteProcess(
        cmd=['ros2', 'launch', 'dobot_bringup', 'dobot_magician_control_system.launch.py'],
        shell=False,
        output='screen'
    )
    
    # Display launch
    display_launch = ExecuteProcess(
        cmd=['ros2', 'launch', 'dobot_description', 'display.launch.py'],
        shell=False,
        output='screen'
    )
    
    # RQT Image View
    rqt_image_view = ExecuteProcess(
        cmd=['ros2', 'run', 'rqt_image_view', 'rqt_image_view'],
        shell=False,
        output='screen'
    )
    
    # Camera node
    camera_node = Node(
        package='smart_tv_factory_solution',  # 패키지 이름
        executable='detect_panel_node',  # 실행 파일 이름
        name='detect_panel_node',  # 노드 이름
        output='screen',  # 출력 설정 (예: 'log', 'screen')
    )
    
    # Dobot control client node
    dobot_client = Node(
        package='smart_tv_factory_solution',
        executable='dobot_control_node',
        name='dobot_control_node',
        output='screen',
    )
    
    return LaunchDescription([
        TimerAction(period=0.0, actions=[dobot_bringup_launch]),
        TimerAction(period=20.0, actions=[display_launch]),
        TimerAction(period=23.0, actions=[camera_node]),
        TimerAction(period=23.0, actions=[rqt_image_view]),
        TimerAction(period=35.0, actions=[dobot_client]),
    ])
