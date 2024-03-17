from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():

    signal_generator = Node(
        package='week1',
        executable='signal',
        output='screen',
    )

    process = Node(
        package='week1',
        executable='signal_processor',
        output='screen',
    )

    rqt_graph_node = Node(
        package='rqt_graph',
        executable='rqt_graph',
        output='screen',
    )

    rqt_plot_node = Node(
        package='rqt_plot',
        executable='rqt_plot',
        output='screen',
    )

    
    l_d = LaunchDescription([signal_generator, process, rqt_graph_node, rqt_plot_node])
    return l_d