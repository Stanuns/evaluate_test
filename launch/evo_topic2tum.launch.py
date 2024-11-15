from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    remappings = [('/robot_pose', '/pose')]

    return LaunchDescription([

        DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true'),

        Node(
            package='evaluate_test', 
            executable='evo_topic2tum', 
            name='evo_topic2tum',
            output='screen', 
            parameters=[{'use_sim_time':use_sim_time}],
            remappings = remappings
        ),
    ])  