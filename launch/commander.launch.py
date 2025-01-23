import os 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, declare_launch_argument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from little_helper_commander import item_tf_broadcaster, velocity_estimator

def generate_launch_description():

    navigtion_config_pkg_dir = get_package_share_directory("mir_navigation") 
    bt_path = os.path.join(navigtion_config_pkg_dir, "behavior_trees/navigate_through_poses_bt.xml")
    

    declare_bt_path = DeclareLaunchArgument('behavior_tree_path', 
                                            default_value= bt_path,
                                            description='the full path to the behavior tree used in navigate through poses')
    commander = Node(package="little_helper_commander",
                     executable="commander",
                     parameters=[{'behavior_tree_path':LaunchConfiguration('behavior_tree_path')}],
                     output='screen')

    item_tf_broadcaster = Node(package="little_helper_commander",
                               executable="item_tf_broadcaster",
                               output="screen")

    velocity_estimator = Node(package="little_helper_commander",
                              executable="velocity_estimator",
                              output="screen")

    return LaunchDescription([declare_bt_path, commander, item_tf_broadcaster, velocity_estimator])
