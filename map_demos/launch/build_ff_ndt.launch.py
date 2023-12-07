from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# Build a map from the small fire fighter bagfile, using ndt_2d

def generate_launch_description():

    start_sync_slam_toolbox_node = Node(
        parameters=[
          get_package_share_directory("map_demos") + '/config/ndt_small_scale.yaml'
        ],
        package='ndt_2d',
        executable='ndt_2d_map_node',
        name='ndt_2d_node',
        output='screen')

    ld = LaunchDescription()
    ld.add_action(start_sync_slam_toolbox_node)

    return ld
