import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. 패키지 및 설정 경로
    pkg_name = 'pc3'
    namespace = 'robot5'
    
    # 2. TF 리매핑 설정 (실기기 통신을 위해 필수)
    # 실제 로봇의 TF가 /robot4/tf 로 오고 있다면 아래와 같이 리매핑합니다.
    remappings = [
        ('/tf', f'/{namespace}/tf'),
        ('/tf_static', f'/{namespace}/tf_static')
    ]

    # 새롭게 추가된 멀티 목표 내비게이션 노드
    pc3_stage5_6_node = Node(
        package=pkg_name,
        executable='pc3_stage5_6_node',
        name='pc3_stage5_6_node',
        namespace=namespace,
        output='screen',
        remappings=remappings,
        parameters=[{'use_sim_time': False}]
    )
    
    # 3. 1&2단계 노드 (섹션 주행)
    stage1_2_node = Node(
        package=pkg_name,
        executable='pc3_stage1_2_node',
        name='pc3_stage1_2_node',
        namespace=namespace,
        output='screen',
        remappings=remappings,
        parameters=[{'use_sim_time': False}]
    )

    # 4. 3&4단계 노드 (YOLO 탐색 및 정밀 이동)
    stage3_4_node = Node(
        package=pkg_name,
        executable='pc3_stage3_4_node',
        name='pc3_stage3_4_node',
        namespace=namespace,
        output='screen',
        remappings=remappings,
        parameters=[{'use_sim_time': False}]
    )
    
    return LaunchDescription([
        pc3_stage5_6_node,
        stage1_2_node,
        stage3_4_node,
    ])
