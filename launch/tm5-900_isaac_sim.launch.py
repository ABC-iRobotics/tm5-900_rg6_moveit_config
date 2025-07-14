############################################################################################### 
#  tm5-900_moveit.launch.py
#   
#  Various portions of the code are based on original source from 
#  The reference: "https://github.com/moveit/moveit2/tree/main/moveit_ros/moveit_servo/launch"
#  and are used in accordance with the following license.
#  "https://github.com/moveit/moveit2/blob/main/LICENSE.txt"
############################################################################################### 

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    TextSubstitution
)

from launch_ros.actions import Node

import xacro
import yaml

from moveit_configs_utils import MoveItConfigsBuilder


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except OSError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    # Declare arguments
    args = []
    length = len(sys.argv)
    if (len(sys.argv) >= 5):
        i = 4
        while i < len(sys.argv):
            args.append(sys.argv[i])
            i = i + 1

    # Configure robot_description
    tm_robot_type = 'tm5-900'
    description_path = 'tm_description'
    xacro_path = 'config/tm5-900.urdf.xacro'
    moveit_config_path = 'tm5-900_rg6_moveit_config'    
    srdf_path = 'config/tm5-900.srdf'
    rviz_path = '/config/moveit.rviz'
    controller_path = 'config/moveit_controllers.yaml'
    ros_controller_path = 'config/ros2_controllers.yaml'
    joint_limits_path = 'config/joint_limits.yaml'

    ros2_control_hardware_type = DeclareLaunchArgument(
        "ros2_control_hardware_type",
        default_value="isaac",
        description="ROS2 control hardware interface type to use for the launch file -- possible values: [mock_components, isaac]",
    )

    # MoveIt Configuration
    moveit_config = (
        MoveItConfigsBuilder(tm_robot_type, package_name="tm5-900_rg6_moveit_config")
        .robot_description(file_path=xacro_path,
            mappings={
                "ros2_control_hardware_type": LaunchConfiguration(
                    "ros2_control_hardware_type"
                )
            },
        )
        .robot_description_semantic(file_path=srdf_path)
        .trajectory_execution(file_path=controller_path)
        .joint_limits(file_path=joint_limits_path)
        .planning_scene_monitor(
                publish_robot_description=True, publish_robot_description_semantic=True
            )
        .planning_pipelines(
                pipelines=["ompl"],
                default_planning_pipeline="ompl",
                load_all=True
            )
        .to_moveit_configs()
    )

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': True},
        ],
    )

    # RViz configuration
    rviz_config_file = (
        get_package_share_directory(moveit_config_path) + rviz_path
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,            
            moveit_config.joint_limits,
            {'use_sim_time': False},
        ],
    )

    # Static TF Node: Publishes world -> base transform
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='log',
        arguments=['0.0', '-0.3', '0.0', '0.0', '0.0', '0.0', 'world', 'base']
    )

    # Robot State Publisher Node: Publishes tf's for the robot
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            moveit_config.robot_description,
        ],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controller_file = get_package_share_directory(moveit_config_path) + ros_controller_path

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[ros2_controller_file],
        remappings=[
            ('/controller_manager/robot_description', '/robot_description'),
        ],
        output='both',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
        ],
        remappings=[
            ('/joint_states', '/joint_command'),
        ]
    )

    tm_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            'tmr_arm_controller', 
            '--controller-manager',
            '/controller_manager',
        ],
    )

    # joint driver
    tm_driver_node = Node(
        package='tm_driver',
        executable='tm_driver',
        # name='tm_driver',
        output='screen',
        arguments=args,
        remappings=[
            ('/joint_states', '/joint_command'),
        ]
    )
    
    #Gripper type argument
    gripper_arg = DeclareLaunchArgument(
        'gripper',
        default_value=TextSubstitution(text='rg6')
    )
    
    #Gripper node interfacing MoveIt2 and Isaac Sim
    rg6_node = Node(
        package='onrobot_rg_control',
        executable='OnRobotRGIsaacSimController',
        name='OnRobotRGIsaacSimController',
        output='screen',
        arguments=[],
        parameters=[{
            '/onrobot/gripper': LaunchConfiguration('gripper'),
        }],
    )
    

    # Launching all the nodes
    return LaunchDescription(
        [
            # ~~~~~~~~~~~~~~~ Arguments ~~~~~~~~~~~~~~~ #
            gripper_arg,
            ros2_control_hardware_type,
           # ~~~~~~~~~~~~~~~~~ Nodes ~~~~~~~~~~~~~~~~~ #
            rviz_node,
            launch_ros.actions.SetParameter(name='use_sim_time', value=True),
            tm_driver_node,
            static_tf,
            robot_state_publisher,
            run_move_group_node,
            ros2_control_node,
            rg6_node,
            #joint_state_broadcaster_spawner,
            #tm_arm_controller_spawner,
        ]
    )
