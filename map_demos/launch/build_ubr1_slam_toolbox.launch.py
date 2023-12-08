from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


# Build a map from the UBR-1 bagfile, using slam_toolbox
def generate_launch_description():

    start_sync_slam_toolbox_node = Node(
        parameters=[
          get_package_share_directory('map_demos') + '/config/slam_toolbox_online_sync.yaml'
        ],
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')

    ld = LaunchDescription()
    ld.add_action(start_sync_slam_toolbox_node)

    return ld
