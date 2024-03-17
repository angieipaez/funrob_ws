from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():

    tarlker_params_node = Node(
        package='basic_comms_params',
        executable='talker_params',
        output='screen',
        parameters=[{'my_param': 'robots!!!'}]
    )

    ld = LaunchDescription([tarlker_params_node])
    return ld