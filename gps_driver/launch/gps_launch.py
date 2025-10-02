from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='Port for the GPS device'
    )
    
    gps_driver_node = Node(
        package='gps_driver',  
        executable='driver',  
        name='gps_driver',   
        output='screen',       
        parameters=[
            {'port': LaunchConfiguration('port')},  
            {'baudrate': 4800},        
            {'sampling_rate': 10.0},
        ]
    )

    launch_description = LaunchDescription([
        port_arg,
        gps_driver_node
    ])
    
    return launch_description
