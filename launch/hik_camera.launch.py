import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 确保这里的包名 'hik_camera' 和你 package.xml 里的一致
    config_dir = os.path.join(get_package_share_directory('hik_camera'), 'config')
    params_file = os.path.join(config_dir, 'camera_params.yaml')
    camera_info_url = 'package://hik_camera/config/camera_info.yaml'

    return LaunchDescription([
        LogInfo(msg=['Loading params from: ', params_file]), # 打印路径方便调试

        DeclareLaunchArgument(name='params_file', default_value=params_file),
        DeclareLaunchArgument(name='camera_info_url', default_value=camera_info_url),
        DeclareLaunchArgument(name='use_sensor_data_qos', default_value='false'),

        Node(
            package='hik_camera',
            executable='hik_camera_node',
            name='hik_camera', # 强制指定节点名，必须与 yaml 中的 /hik_camera 对应
            output='screen',
            emulate_tty=True,
            parameters=[
                LaunchConfiguration('params_file'), # 加载 camera_params.yaml
                {
                    'camera_info_url': LaunchConfiguration('camera_info_url'),
                    'use_sensor_data_qos': LaunchConfiguration('use_sensor_data_qos'),
                }
            ],
        )
    ])