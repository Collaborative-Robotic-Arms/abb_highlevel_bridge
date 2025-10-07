from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # --- 1. Define the SRDF path as a launch argument ---
    # We declare it here, but set a fixed default value for convenience.
    # Define the name of the package containing the SRDF file
#    (This package name MUST match the directory in the path: irb120_ros2_moveit2)
    SRDF_PACKAGE_NAME = 'irb120_ros2_moveit2' 

    # 1. Get the install location (share directory) of the SRDF package
    pkg_share_dir = get_package_share_directory(SRDF_PACKAGE_NAME)

    # 2. Construct the file path relative to the package share directory
    srdf_file_path = os.path.join(pkg_share_dir, 'config', 'irb120.srdf')

    # Use this new relative path in the DeclareLaunchArgument
    srdf_path_arg = DeclareLaunchArgument(
        'srdf_path',
        default_value=srdf_file_path,
        description='Relative path to the SRDF file published from the package share directory.'
    )

    # # --- 2. Configure the C++ Node ---
    # # NOTE: Replace 'your_control_package' with the actual package name 
    # # where the 'semantic_publisher' executable is installed.
    semantic_publisher_node = Node(
        package='abb_highlevel_bridge', # <-- CHANGE THIS
        executable='semantic_publisher',
        name='semantic_publisher',
        output='screen',
        parameters=[{
            # Passing the launch argument value to the node's parameter
            'srdf_path': LaunchConfiguration('srdf_path')
        }]
    )

    abb_inverse_node = Node(
        package='abb_highlevel_bridge', 
        executable='inverse_kinematics',
        name='abb_inverse_control',
        output='screen',
        parameters=[]
    )

    gripper_node = Node(
        package='abb_highlevel_bridge',
        executable='abb_endeffector_control_client',
        name='gripper_control_client',
        output='screen',
        parameters=[]
    )

    return LaunchDescription([
        srdf_path_arg,
        abb_inverse_node,
        semantic_publisher_node,
        gripper_node
    ])
