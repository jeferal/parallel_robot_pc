import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from datetime import datetime
from os import mkdir
from os.path import expanduser


def generate_launch_description():

    home = expanduser("~")

    now = datetime.now()

    now_str = now.strftime("%d_%m_%Y-%H_%M_%S")
    data_folder = home + "/ros2_eloquent_ws/parallel_robot_pc/exp_data/exp_" + now_str

    try:
        mkdir(data_folder)
    except OSError:
        print("Could not create the directory")
    else:
        print("Directory created: %s" % data_folder)
    
    pr_storage = ComposableNodeContainer(
            node_name='pr_storage_container',
            node_namespace='',
            package='rclcpp_components',
            node_executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='pr_storage',
                    node_plugin='pr_storage::SingleRecorder',
                    node_name='position_recorder',
                    remappings=[
                        ("sub", "joint_position"),
                    ],
                    parameters=[
                        {"data_name": "joint_position"},
                        {"data_dir_path": data_folder},
                    ]
                ),
                ComposableNode(
                    package='pr_storage',
                    node_plugin='pr_storage::SingleRecorder',
                    node_name='ref_pose_recorder',
                    remappings=[
                        ("sub", "ref_pose"),
                    ],
                    parameters=[
                        {"data_name": "ref_pose"},
                        {"data_dir_path": data_folder},
                    ]
                ),
                ComposableNode(
                    package='pr_storage',
                    node_plugin='pr_storage::SingleRecorder',
                    node_name='velocity_recorder',
                    remappings=[
                        ("sub", "joint_velocity"),
                    ],
                    parameters=[
                        {"data_name": "joint_velocity"},
                        {"data_dir_path": data_folder},
                    ]
                ),
                ComposableNode(
                    package='pr_storage',
                    node_plugin='pr_storage::SingleRecorder',
                    node_name='control_action_recorder',
                    remappings=[
                        ("sub", "control_action"),
                    ],
                    parameters=[
                        {"data_name": "control_action"},
                        {"data_dir_path": data_folder},
                    ]
                ),
                ComposableNode(
                    package='pr_storage',
                    node_plugin='pr_storage::SingleRecorder',
                    node_name='x_coord_recorder',
                    remappings=[
                        ("sub", "x_coord"),
                    ],
                    parameters=[
                        {"data_name": "x_coord"},
                        {"data_dir_path": data_folder},
                    ]
                ),
                ComposableNode(
                    package='pr_storage',
                    node_plugin='pr_storage::SingleRecorder',
                    node_name='q_grav_recorder',
                    remappings=[
                        ("sub", "q_grav"),
                    ],
                    parameters=[
                        {"data_name": "q_grav"},
                        {"data_dir_path": data_folder},
                    ]
                ),
                ComposableNode(
                    package='pr_storage',
                    node_plugin='pr_storage::MocapRecorder',
                    node_name='mocap_recorder',
                    remappings=[
                        ("sub", "x_coord_mocap_error"),
                    ],
                    parameters=[
                        {"data_name": "x_mocap"},
                        {"data_dir_path": data_folder},
                    ]
                ),
            ],
            output='screen',
    )

    return launch.LaunchDescription([pr_storage])