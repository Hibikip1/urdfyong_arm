import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch Gazebo + ros2_control + MoveIt2 + RViz2 for urdfyong arm.

    Brings up the full simulation-to-planning pipeline:
      1. Gazebo with empty world
      2. Spawns the urdfyong robot (triggers ros2_control via URDF plugin)
      3. Loads joint_state_broadcaster + joint_trajectory_controller
      4. Starts move_group (MoveIt2 planning node)
      5. Starts RViz2 with MoveIt MotionPlanning panel

    The user can then drag the interactive marker in RViz2,
    click "Plan & Execute", and watch the arm move in Gazebo.
    """

    # ── Paths ────────────────────────────────────────────────────────
    pkg_urdfyong = get_package_share_directory('urdfyong')
    pkg_moveit_cfg = get_package_share_directory('urdfyong_moveit_config')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    urdf_path = os.path.join(pkg_urdfyong, 'urdf', 'urdfyong.urdf')
    rviz_path = os.path.join(pkg_moveit_cfg, 'config', 'moveit.rviz')

    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    robot_description = {'robot_description': robot_desc}

    # ── Launch arguments ────────────────────────────────────────────
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # ── Gazebo ─────────────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'verbose': 'false'}.items(),
    )

    # ── Robot State Publisher ──────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    # ── Spawn robot in Gazebo ─────────────────────────────────────
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_urdfyong',
        arguments=[
            '-entity', 'urdfyong',
            '-topic', '/robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '0.0',
        ],
        output='screen',
    )

    # ── Controller spawners ────────────────────────────────────────
    # These wait for the controller_manager service (created by
    # gazebo_ros2_control plugin) then load + activate controllers.
    controller_spawner_jsb = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_state_broadcaster_spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
    )

    controller_spawner_jtc = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_trajectory_controller_spawner',
        arguments=[
            'joint_trajectory_controller',
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
    )

    # ── MoveGroup ──────────────────────────────────────────────────
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_moveit_cfg, 'launch', 'move_group.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # ── RViz2 ──────────────────────────────────────────────────────
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_path],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ── Orchestration ──────────────────────────────────────────────
    # Spawn controllers after the robot is spawned in Gazebo.
    # Delay move_group and rviz slightly to ensure Gazebo is up.
    spawn_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[controller_spawner_jsb, controller_spawner_jtc],
        )
    )

    delayed_move_group = TimerAction(
        period=4.0,
        actions=[move_group],
    )

    delayed_rviz = TimerAction(
        period=5.0,
        actions=[rviz2],
    )

    # ── Launch Description ─────────────────────────────────────────
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock',
        ),
        gazebo,
        robot_state_publisher,
        spawn_entity,
        spawn_controllers,
        delayed_move_group,
        delayed_rviz,
    ])
