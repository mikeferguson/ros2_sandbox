from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


# Build a map from the UBR-1 bagfile, using ndt_2d
def generate_launch_description():

    start_sync_slam_toolbox_node = Node(
        parameters=[
          get_package_share_directory('map_demos') + '/config/ndt_large_scale.yaml'
        ],
        package='ndt_2d',
        executable='ndt_2d_map_node',
        name='ndt_2d_node',
        remappings=[('scan', 'base_scan')],
        output='screen')

    ld = LaunchDescription()
    ld.add_action(start_sync_slam_toolbox_node)

    return ld
