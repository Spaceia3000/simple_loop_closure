import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()
    package_dir = get_package_share_directory('simple_loop_closure')
    default_params_file = os.path.join(package_dir, 'config', 'cohemo_livo.yaml')
    # FAST-LIVO2's metric cloud is in camera_init and has the exact same
    # measurement stamp as /aft_mapped_to_init. /cloud_registered is not an
    # equivalent input because it is stamped with the node clock.
    default_odometry_topic = "/aft_mapped_to_init"
    default_cloud_topic = "/cloud_registered_metric"

    arg_loop_closure_params_file = LaunchConfiguration('loop_closure_params_file')
    arg_odometry_topic = LaunchConfiguration('odometry_topic')
    arg_cloud_topic = LaunchConfiguration('cloud_topic')

    ld.add_action(DeclareLaunchArgument('loop_closure_params_file', default_value=str(default_params_file),
                                        description='name or path to the parameters file to use.'))
    ld.add_action(DeclareLaunchArgument('odometry_topic', default_value=default_odometry_topic))
    ld.add_action(DeclareLaunchArgument('cloud_topic', default_value=default_cloud_topic))

    remappings=[
        ('/odometry', arg_odometry_topic),
        ('/cloud', arg_cloud_topic)
    ]

    node_loop_closure = Node(
        package='simple_loop_closure',
        executable='simple_loop_closure_node',
        name='simple_loop_closure',
        output='both',
        parameters=[arg_loop_closure_params_file],
        remappings=remappings,
    )
    ld.add_action(node_loop_closure)

    return ld
