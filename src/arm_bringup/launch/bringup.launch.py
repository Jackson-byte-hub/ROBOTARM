from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    robot_description = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('arm_description'),
            'urdf',
            'arm.urdf.xacro'
        ])
    ])

    return LaunchDescription([

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),

        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'robot_description': robot_description},
                PathJoinSubstitution([
                    FindPackageShare('arm_bringup'),
                    'config',
                    'controllers.yaml'
                ])
            ],
            output='screen'
        ),
    ])
