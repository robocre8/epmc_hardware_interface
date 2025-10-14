import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    # delare any path variable
    description_pkg_path = get_package_share_directory('description_pkg')
    base_pkg_path = get_package_share_directory('robot_base_pkg')

    robot_controllers = os.path.join(base_pkg_path,'config','robot_base_controller.yaml')

    #--------------------------------------------------------------------------

    # create needed nodes or launch files
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(description_pkg_path,'launch','rsp.launch.py')]), 
        launch_arguments={'use_sim_time': 'False',
                          'run_gz_sim': 'False'}.items(),
        )
    
    # see -> https://github.com/ros-controls/ros2_control_demos/blob/humble/example_2/bringup/launch/diffbot.launch.py
    # see -> https://control.ros.org/master/doc/ros2_control/controller_manager/doc/userdoc.html
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_controller",
            "--param-file",
            robot_controllers,
            "--controller-ros-args",
            """
            -r /diff_drive_controller/cmd_vel:=/cmd_vel
            -r /diff_drive_controller/odom:=/odom
            """
        ]
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    start_diff_drive_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_controller_spawner],
        )
    )


    # Create the launch description and populate
    ld = LaunchDescription()

    # Add the nodes to the launch description
    ld.add_action(rsp_launch)
    ld.add_action(controller_manager)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(start_diff_drive_controller_spawner_after_joint_state_broadcaster_spawner)

    return ld      # return (i.e send) the launch description for excecution