#!/usr/bin/env -S python3

import os

from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros

def generate_launch_description():
    return launch.LaunchDescription(
        generate_declare_launch_arguments()
        + generate_launch_nodes()
    )

def generate_declare_launch_arguments():
    this_pkg_share_dir = get_package_share_directory('go1_description')

    return [
        launch.actions.DeclareLaunchArgument(
            'namespace',
            default_value = [''],
            description = 'Namespace (string)'
        ),
        launch.actions.DeclareLaunchArgument(
            'robot_model_file',
            default_value = ['unitree_go1.urdf.xacro'],
            description = 'Robot model file (string)'
        ),
        launch.actions.DeclareLaunchArgument(
            'robot_model_path',
            default_value = [
                os.path.join(
                    this_pkg_share_dir,
                    'models',
                    'urdf'
                )
            ],
            description = 'Robot model file path (string)'
        ),
        launch.actions.DeclareLaunchArgument(
            'joint_state_publisher_config_file',
            default_value = [
                os.path.join(
                    this_pkg_share_dir,
                    'config',
                    'joint_state.yaml'
                )
            ],
            description = 'Joint state publisher configulation file of full path (string)'
        ),
        launch.actions.DeclareLaunchArgument(
            'use_sim_time',
            default_value = ['false'],
            description = 'Use simulation time (boolean)'
        ),
        launch.actions.DeclareLaunchArgument(
            'use_rviz',
            default_value = ['false'],
            description = 'Using rviz2 (boolean)'
        ),
        launch.actions.DeclareLaunchArgument(
            'rviz_config_file',
            default_value = [
                os.path.join(
                    this_pkg_share_dir,
                    'rviz',
                    'unitree_go1.rviz'
                )
            ],
            condition = launch.conditions.IfCondition(
                launch.substitutions.LaunchConfiguration('use_rviz')
            ),
            description = 'RViz2 config file of full path (string)'
        )
    ]

def generate_launch_nodes():
    # TODO Output parameter from declare launch arguemnt
    output = 'screen'

    urdf_file = launch.substitutions.PathJoinSubstitution([
        launch.substitutions.LaunchConfiguration('robot_model_path'),
        launch.substitutions.LaunchConfiguration('robot_model_file')
    ])

    use_sim_time = {
        'use_sim_time': launch.substitutions.LaunchConfiguration(
            'use_sim_time'
        )
    }
    robot_description = {
        'robot_description': launch.substitutions.Command([
            'xacro ',
            urdf_file
        ])
    }

    exit_event = launch.actions.EmitEvent(
        event = launch.events.Shutdown()
    )

    return [
        launch.actions.GroupAction(actions = [
            launch_ros.actions.PushRosNamespace(
                namespace = launch.substitutions.LaunchConfiguration('namespace')
            ),
            launch_ros.actions.Node(
                package = 'robot_state_publisher',
                executable = 'robot_state_publisher',
                name = 'robot_state_publisher',
                output = output,
                parameters = [
                    use_sim_time,
                    robot_description
                ],
                on_exit = exit_event
            ),
            launch_ros.actions.Node(
                package = 'joint_state_publisher',
                executable = 'joint_state_publisher',
                name = 'joint_state_merger',
                output = output,
                parameters = [
                    use_sim_time,
                    launch.substitutions.LaunchConfiguration(
                        'joint_state_publisher_config_file'
                    )
                ],
                on_exit = exit_event
            ),
            launch_ros.actions.Node(
                package = 'rviz2',
                executable = 'rviz2',
                name = 'rviz2',
                output = output,
                parameters = [
                    use_sim_time,
                ],
                arguments = [
                    '-d',
                    launch.substitutions.LaunchConfiguration(
                        'rviz_config_file'
                    )
                ],
                condition = launch.conditions.IfCondition(
                    launch.substitutions.LaunchConfiguration(
                        'use_rviz'
                    )
                )
            ),
        ])
    ]

