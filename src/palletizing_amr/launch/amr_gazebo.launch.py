import os

from ament_index_python.packages import (
    get_package_share_directory,
    get_package_prefix
)

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    AppendEnvironmentVariable,
    TimerAction
)

from launch.launch_description_sources import (
    PythonLaunchDescriptionSource
)

from launch_ros.actions import Node


def generate_launch_description():

    pkg_name = 'palletizing_amr'
    pkg_share = get_package_share_directory(pkg_name)

    # Gazebo resource path so meshes can be found
    workspace_share_dir = os.path.join(
        get_package_prefix(pkg_name),
        'share'
    )

    # URDF file path
    urdf_file = os.path.join(
        pkg_share,
        'urdf',
        'palletizing_amr.urdf'
    )

    # Read URDF
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Tell Gazebo where model resources are
    set_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        workspace_share_dir
    )

    # Robot State Publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_desc}
        ]
    )

    # Start Gazebo Sim (empty world for testing)
    pkg_ros_gz_sim = get_package_share_directory(
        'ros_gz_sim'
    )

    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_ros_gz_sim,
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': 'empty.sdf -r'
        }.items()
    )

    # Spawn robot
    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'palletizing_amr',
            '-topic', 'robot_description',
            '-z', '0.15'     # higher spawn for safe physics start
        ],
        output='screen'
    )

    # Delay spawn so Gazebo fully starts first
    delayed_spawn = TimerAction(
        period=3.0,
        actions=[spawn_node]
    )

    return LaunchDescription([
        set_resource_path,
        rsp_node,
        gazebo_node,
        delayed_spawn
    ])
