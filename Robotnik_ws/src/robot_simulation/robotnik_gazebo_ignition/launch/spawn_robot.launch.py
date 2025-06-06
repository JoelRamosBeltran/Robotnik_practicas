# Copyright (c) 2022, Robotnik Automation S.L.L.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Robotnik Automation S.L.L. nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL Robotnik Automation S.L.L. BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
import os
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue
from robotnik_common.launch import ExtendedArgument, AddArgumentParser
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, RegisterEventHandler, TimerAction)


def generate_launch_description():

    ld = LaunchDescription()
    add_to_launcher = AddArgumentParser(ld)

    arg = ExtendedArgument(
        name='namespace',
        description='Namespace',
        default_value='robot',
        use_env=True,
        environment='NAMESPACE',
    )
    add_to_launcher.add_arg(arg)

    arg = ExtendedArgument(
        name='robot',
        description='Robot model (rbvogui, rbkairos, rbtheron, rbsummit)',
        default_value='rbvogui',
        use_env=True,
        environment='ROBOT',
    )
    add_to_launcher.add_arg(arg)
    robot = LaunchConfiguration('robot')
    
    arg = ExtendedArgument(
        name='robot_model',
        description='Robot type variation (rbvogui, rbvogui_6w, rbvogui_ackermann)',
        default_value=robot,
        use_env=True,
        environment='ROBOT_MODEL',
    )
    add_to_launcher.add_arg(arg)
    robot_model = LaunchConfiguration('robot_model')

    arg = ExtendedArgument(
        name='robot_xacro_file',
        description='Name of the xacro file',
        default_value=[robot, '/', robot_model, '.urdf.xacro'],
        use_env=True,
        environment='ROBOT_XACRO_FILE',
    )
    add_to_launcher.add_arg(arg)

    robot_xacro_file = LaunchConfiguration('robot_xacro_file')
    arg = ExtendedArgument(
        name='robot_xacro_path',
        description='Path to the xacro file',
        default_value=[FindPackageShare('robot_description'), '/robots/', robot_xacro_file],
        use_env=True,
        environment='ROBOT_XACRO_PATH',
    )
    add_to_launcher.add_arg(arg)

    arg = ExtendedArgument(
        name='x',
        description='x position in world',
        default_value='0.0',
    )
    add_to_launcher.add_arg(arg)

    arg = ExtendedArgument(
        name='y',
        description='y position in world',
        default_value='0.0',
    )
    add_to_launcher.add_arg(arg)

    arg = ExtendedArgument(
        name='z',
        description='z position in world',
        default_value='0.0',
    )
    add_to_launcher.add_arg(arg)
    params = add_to_launcher.process_arg()

    robot_dir = os.path.join(get_package_share_directory('robot_description'), 'launch')

    robot_state = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(robot_dir, 'robot_description.launch.py')
            ),
            launch_arguments={
                'verbose': 'false',
                'robot_xacro_file': robot_xacro_file,
                'namespace': params['namespace'],
                'gazebo_ignition': 'true',
            }.items(),
    )

    ld.add_action(robot_state)

    robot_spawner = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', [params['namespace'], '/', params['robot']],
                '-topic', "robot_description",
                '-robot_namespace', params['namespace'],
                '-x', params['x'],
                '-y', params['y'],
                '-z', params['z'],
            ],
            output='screen',
            namespace=params['namespace']
    )
    ld.add_action(robot_spawner)
    bridge_params = [get_package_share_directory('robotnik_gazebo_ignition'),'/config/', robot,'/bridge.yaml']

    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
         parameters=[
            {'config_file': bridge_params,
            'expand_gz_topic_names':True},
        ],
        namespace=params['namespace']
    )

    # ros_gz_image_bridge = Node(
    #     package="ros_gz_image",
    #     executable="image_bridge",
    #     arguments=[
    #         "/robot/front_rgbd_camera/color/image_raw", 
    #         "/robot/rear_rgbd_camera/color/image_raw"
    #         #"/robot/front_rgbd_camera/ired1/image_raw", 
    #         #"/robot/rear_rgbd_camera/ired1/image_raw",
    #         #"/robot/front_rgbd_camera/ired2/image_raw", 
    #         #"/robot/rear_rgbd_camera/ired2/image_raw",
    #         #"/robot/front_rgbd_camera/depth/image_raw",
    #         #"/robot/rear_rgbd_camera/depth/image_raw"
    #     ],
    #     namespace=params['namespace']
    # )
    # ld.add_action(ros_gz_image_bridge)

    # controller_dir = os.path.join(get_package_share_directory('robotnik_controller'), 'launch')

    
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        namespace=params['namespace']
    )
    ld.add_action(joint_state_broadcaster)

    robotnik_controller= Node(
        package='controller_manager',
        executable='spawner',
        arguments=['robotnik_base_controller'],
        output='screen',
        emulate_tty=True,
        namespace=params['namespace']
    )

    init_robotnik_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[
                LogInfo(msg='Joint States spawned'),
                robotnik_controller
            ]
        )
    )
    ld.add_action(init_robotnik_controller)
    
    init_param_bridge = RegisterEventHandler(
        OnProcessExit(
            target_action=robotnik_controller,
            on_exit=[
                LogInfo(msg='Joint States spawned'),
                ros_gz_bridge
            ]
        )
    )
    ld.add_action(init_param_bridge)

    return ld
