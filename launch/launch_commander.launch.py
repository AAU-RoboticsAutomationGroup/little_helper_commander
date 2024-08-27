
import os 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, declare_launch_argument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from little_helper_commander import item_tf_broadcaster
import little_helper_commander

def generate_launch_description():
    navigtion_config_pkg_dir = get_package_share_directory("mir_navigation") 
    bt_path = os.path.join(navigtion_config_pkg_dir, "behavior_trees/navigate_through_poses_bt.xml")

    declare_initial_pose = DeclareLaunchArgument('initial_pose', description='the initial pose of the robot in [x, y, w, z]')

    declare_item_position = DeclareLaunchArgument('item_position', description='the position of the item in the map frame in [x, y, z] format')

    declare_goal_pose = DeclareLaunchArgument('goal_pose', description='the goal pose of the path in [x, y, w, z]')

    declare_bt_path = DeclareLaunchArgument('behavior_tree_path', 
                                            default_value= bt_path,
                                            description='the full path to the behavior tree used in navigate through poses')

    navigator_commander = Node(package='little_helper_commander',
                               executable='commander',
                               parameters=[{'initial_pose': LaunchConfiguration('initial_pose'),
                                           'goal_pose':LaunchConfiguration('goal_pose'),
                                           'item_position':LaunchConfiguration('item_position'),
                                           'behavior_tree_path':LaunchConfiguration('behavior_tree_path')}],
                               output='screen')

    item_tf_broadcaster = Node(package='little_helper_commander',
                               executable='item_tf_broadcaster',
                               parameters=[{'item_position':LaunchConfiguration('item_position')}],
                               output='screen')


    ld = LaunchDescription([declare_initial_pose,
                            declare_item_position,
                            declare_goal_pose,
                            declare_bt_path,
                            navigator_commander,
                            item_tf_broadcaster
                            ])
    return ld
    

