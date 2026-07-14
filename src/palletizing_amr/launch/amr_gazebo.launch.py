import os

from ament_index_python.packages import (
    get_package_share_directory,
    get_package_prefix
)

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    AppendEnvironmentVariable,
    ExecuteProcess,
    TimerAction
)

from launch.launch_description_sources import (
    PythonLaunchDescriptionSource
)

from launch_ros.actions import Node


def generate_launch_description():

    pkg_name = 'palletizing_amr'
    pkg_share = get_package_share_directory(pkg_name)

    world_file = os.path.join(
        pkg_share,
        "worlds",
        "warehouse.sdf"
    )

    # Gazebo resource path so meshes can be found
    workspace_share_dir = os.path.join(
        get_package_prefix(pkg_name),
        'share'
    )

    resource_paths = [
        workspace_share_dir,
        os.path.join(pkg_share, "models"),
    ]

    # URDF file path
    urdf_file = os.path.join(
        pkg_share,
        'urdf',
        'palletizing_amr.urdf'
    )

    # Read URDF
    with open(urdf_file, 'r', encoding='utf-8') as infp:
        robot_desc = infp.read()

    # Tell Gazebo where model resources are
    set_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.pathsep.join(resource_paths)
    )

    # Robot State Publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_desc},
            {'use_sim_time': True}
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
            'gz_args': f'-r {world_file}'
        }.items()
    )

    # Spawn robot
    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'palletizing_amr',
            '-topic', 'robot_description',
            '-x', '2.50',
            '-y', '14.73',
            '-z', '1.0',
            '-Y', '-1.5708'
        ],
        output='screen'
    )

    # Delay spawn so Gazebo fully starts first (Robot drops at 10 seconds)
    delayed_spawn = TimerAction(
        period=10.0,
        actions=[spawn_node]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_base", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    delayed_joint_broadcaster = TimerAction(
        period=13.0,
        actions=[joint_state_broadcaster_spawner]
    )

    delayed_diff_drive = TimerAction(
        period=15.0,
        actions=[diff_drive_spawner]
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # scan_bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     arguments=['/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
    #     output='screen'
    # )

    pointcloud_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'
        ],
        output='screen'
    )

    rviz_config = os.path.join(
        pkg_share,
        'rviz',
        'amr.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    image_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image'],
        output='screen'
    )

    camera_info_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'],
        output='screen'
    )

    delayed_rqt = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='rqt_image_view',
                executable='rqt_image_view',
                arguments=['/camera/image_raw'],
                output='screen'
            )
        ]
    )

    cmd_vel_relay = ExecuteProcess(
        cmd=['python3', os.path.join(pkg_share, 'scripts', 'cmd_vel_relay.py')],
        output='screen'
    )

    forklift_effort_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forklift_effort_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    delayed_forklift_effort = TimerAction(
        period=17.0,
        actions=[forklift_effort_spawner]
    )

    forklift_pid_node = Node(
        package='palletizing_amr',
        executable='forklift_position_pid',
        output='screen'
    )

    delayed_forklift_pid = TimerAction(
        period=18.0,   # after the effort controller is active
        actions=[forklift_pid_node]
    )

    return LaunchDescription([
        set_resource_path,
        rsp_node,
        gazebo_node,
        clock_bridge,
        # scan_bridge,
        pointcloud_bridge,
        rviz_node,
        image_bridge,
        camera_info_bridge,
        cmd_vel_relay,
        delayed_spawn,
        delayed_joint_broadcaster,  # Notice we are using the delayed variable now
        delayed_diff_drive,         # Notice we are using the delayed variable now
        delayed_forklift_effort,
        delayed_forklift_pid,
        delayed_rqt
    ])
