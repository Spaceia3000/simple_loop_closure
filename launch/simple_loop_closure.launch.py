import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('simple_loop_closure')
    mode_arg = LaunchConfiguration('mode')
    params_arg = LaunchConfiguration('loop_closure_params_file')
    odometry_arg = LaunchConfiguration('odometry_topic')
    cloud_arg = LaunchConfiguration('cloud_topic')

    def launch_backend(context):
        mode = mode_arg.perform(context).strip().lower()
        if mode not in ('lio', 'livo'):
            raise ValueError(f"Unsupported SimpleLoopClosure mode {mode!r}; use lio or livo")

        override = params_arg.perform(context).strip()
        params_file = override or os.path.join(package_dir, 'config', f'ugv_lsi_{mode}.yaml')
        if not os.path.isfile(params_file):
            raise FileNotFoundError(f"SimpleLoopClosure parameters not found: {params_file}")

        return [Node(
            package='simple_loop_closure',
            executable='simple_loop_closure_node',
            name='simple_loop_closure',
            output='both',
            parameters=[params_file],
            remappings=[
                ('/odometry', odometry_arg),
                ('/cloud', cloud_arg),
            ],
        )]

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='livo',
                              description='Sensor mode: lio or livo.'),
        DeclareLaunchArgument('loop_closure_params_file', default_value='',
                              description='Optional YAML override; otherwise load config/ugv_lsi_<mode>.yaml from installed package.'),
        DeclareLaunchArgument('odometry_topic', default_value='/aft_mapped_to_init'),
        DeclareLaunchArgument('cloud_topic', default_value='/cloud_registered_metric'),
        OpaqueFunction(function=launch_backend),
    ])
