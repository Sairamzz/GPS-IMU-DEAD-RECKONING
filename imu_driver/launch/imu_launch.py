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
    
    imu_driver_node = Node(
        package='imu_driver',  
        executable='driver',  
        name='imu_driver',   
        output='screen',       
        parameters=[
            {'port': LaunchConfiguration('port')},  
            {'baudrate': 115200},        
            {'output_frequency': 40},
        ]
    )

    launch_description = LaunchDescription([
        port_arg,
        imu_driver_node
    ])
    
    return launch_description