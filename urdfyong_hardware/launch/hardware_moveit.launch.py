import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    """Launch real hardware + ros2_control + MoveIt2 + RViz2 for urdfyong arm.

    Brings up the full hardware-to-planning pipeline:
      1. Controller Manager with DmMotorHardware (real motors via USB2CANFD)
      2. JointStateBroadcaster + JointTrajectoryController
      3. move_group (MoveIt2 planning node)
      4. RViz2 with MoveIt MotionPlanning panel
    """

    # ── Paths ────────────────────────────────────────────────────────
    pkg_urdfyong = get_package_share_directory('urdfyong')
    pkg_hw = get_package_share_directory('urdfyong_hardware')
    pkg_moveit_cfg = get_package_share_directory('urdfyong_moveit_config')
    urdf_path = os.path.join(pkg_urdfyong, 'urdf', 'urdfyong_hardware.urdf')
    rviz_path = os.path.join(pkg_moveit_cfg, 'config', 'moveit.rviz')

    # ── MoveIt configs (for planning + RViz kinematics) ───────────
    moveit_config = (
        MoveItConfigsBuilder("urdfyong", package_name="urdfyong_moveit_config")
        .to_moveit_configs()
    )

    # ── Hardware URDF (for RSP + controller_manager) ──────────────
    with open(urdf_path, 'r') as f:
        hw_robot_desc = f.read()

    hw_robot_description = {'robot_description': hw_robot_desc}

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # ── RViz params: merge all MoveIt config dicts into one ───────
    rviz_params = {}
    rviz_params.update(moveit_config.robot_description)
    rviz_params.update(moveit_config.robot_description_semantic)
    rviz_params.update(moveit_config.robot_description_kinematics)
    rviz_params.update(moveit_config.joint_limits)
    rviz_params['use_sim_time'] = use_sim_time

    # ── Controller Manager params ─────────────────────────────────
    # IMPORTANT: Use a YAML file path (not a dict).  ROS2 loads
    # YAML-file params via --ros-args --params-file at node-init time
    # (before the CM reads them), whereas dict params are applied
    # via set_parameters() after the node is already running.
    # Both top-level keys are loaded into the controller_manager node.
    cm_params_yaml = os.path.join(pkg_hw, 'config', 'hardware_controllers.yaml')

    # ── Robot State Publisher (hardware URDF) ─────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[hw_robot_description, {'use_sim_time': use_sim_time}],
    )

    # ── Controller Manager (standalone, uses DmMotorHardware) ─────
    # IMPORTANT: Do NOT name it "controller_manager" — that triggers
    # ros2_control issue #1684 where multiple CM instances spawn
    # and parameters are lost.
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='urdfyong_controller_manager',
        output='screen',
        parameters=[
            hw_robot_description,
            cm_params_yaml,
            {'use_sim_time': use_sim_time},
        ],
    )

    # ── Controller spawners ────────────────────────────────────────
    controller_spawner_jsb = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_state_broadcaster_spawner',
        arguments=[
            'joint_state_broadcaster',
            '-c', '/urdfyong_controller_manager',
            '--controller-manager-timeout', '30.0',
        ],
        output='screen',
    )

    # ── Trajectory executor (replaces JTC) ─────────────────────────
    # Humble bug: lifecycle sub-nodes don't expose functional param
    # services, so JTC can never receive its config.  This standalone
    # Python node implements /follow_joint_trajectory directly,
    # publishing interpolated position commands to /position_commands.
    # The hardware plugin subscribes to that topic and drives motors.
    trajectory_executor = Node(
        package='urdfyong_hardware',
        executable='trajectory_executor.py',
        name='trajectory_executor',
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

    # ── RViz2 (with full MoveIt configs for interactive marker) ────
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_path],
        output='screen',
        parameters=[rviz_params],
    )

    # ── Orchestration ──────────────────────────────────────────────
    delayed_spawner_jsb = TimerAction(period=3.0, actions=[controller_spawner_jsb])
    delayed_trajectory_executor = TimerAction(period=4.0, actions=[trajectory_executor])
    delayed_move_group = TimerAction(period=5.0, actions=[move_group])
    delayed_rviz = TimerAction(period=6.0, actions=[rviz2])

    # ── Launch Description ─────────────────────────────────────────
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock (must be false for real hardware)',
        ),
        robot_state_publisher,
        controller_manager,
        delayed_spawner_jsb,
        delayed_trajectory_executor,
        delayed_move_group,
        delayed_rviz,
    ])
