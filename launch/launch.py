from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Constants
    pkg_name = "umiusi_control_demo"

    # Launch configuration
    robot_description_path = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution(
            [FindPackageShare(pkg_name), "urdf", "umiusi.urdf.xacro"]),
    ]
    )

    robot_controllers = PathJoinSubstitution([
        FindPackageShare(pkg_name),
        "config",
        "controllers.yaml"
    ]
    )

    # Launch nodes
    return LaunchDescription([
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[robot_controllers],
            output="both"
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster",
                       "umiusi_controller", "--param-file", robot_controllers],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description_path}],
            output="both"
        ),
    ])
