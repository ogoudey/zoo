from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from ros_gz_bridge.actions import RosGzBridge

from launch_ros.actions import Node

def generate_launch_description():

    bridge_name = LaunchConfiguration('bridge_name')
    config_file = LaunchConfiguration('config_file')

    declare_bridge_name_cmd = DeclareLaunchArgument(
        'bridge_name', description='Name of ros_gz_bridge node'
    )

    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file', description='YAML config file'
    )
    
    gz_interface_node = Node(package='gz_interface', executable='gz_interface', name='gz_interface')
    

    
    # Create the launch description and populate
    ld = LaunchDescription([
        RosGzBridge(
            bridge_name=LaunchConfiguration('bridge_name'),
            config_file=LaunchConfiguration('config_file'),
        ),
        
    ])

    # Declare the launch options
    #ld.add_action(gz_interface_node)
    ld.add_action(declare_bridge_name_cmd)
    ld.add_action(declare_config_file_cmd)

    return ld
