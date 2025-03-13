import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import launch_ros.descriptions

def generate_launch_description():
    package_name = "robotnik_gazebo_ignition"  # Replace with your actual package name
    desc_package_name = get_package_share_directory(package_name)
    desc_package_robot_description = get_package_share_directory("robot_description")
    desc_package_sensors = get_package_share_directory("robotnik_sensors")
    
    # Set ignition resource path
    ign_resource_path = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=[
            os.path.join(desc_package_name, "worlds"),
            ":" + str(Path(desc_package_name).parent.resolve()),
            ":" + str(Path(desc_package_sensors).parent.resolve()),
            ":" + str(Path(desc_package_robot_description).parent.resolve()),
        ],
    )

    use_sim_time = LaunchConfiguration("use_sim_time")

    xacro_file = os.path.join(desc_package_robot_description, "robots/rbrobout", "rbrobout.urdf.xacro")

    robot_description_config = Command(
        ["xacro ", xacro_file, " sim_mode:=", use_sim_time]
    )

    # Create a robot_state_publisher node
    params = {
        "robot_description": launch_ros.descriptions.ParameterValue(
            robot_description_config, value_type=str
        ),
        "use_sim_time": use_sim_time,
    }

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[params],
    )

    world = os.path.join(
        desc_package_name, "worlds", "empty_world.sdf"
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={"gz_args": f"-r -v 4 {world}"}.items(),
    )

    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "groundhog",
            "-topic",
            "/robot_description",
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "1.4",
        ],
        output="screen",
    )

    # Add ros_gz_bridge to bridge topics automatically
    bridge_imu = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', 
            '/imu/data@sensor_msgs/msg/Imu@ignition.msgs.IMU'
        ],
        output="screen"
    )
    bridge_front_lidar = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', 
            '/front_laser/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '--ros-args','-r','/front_laser/scan:=/laser_scan'
        ],
        output="screen"
    )

    # Launch!
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time", default_value="true", description="Use sim time if true"
            ),
            ign_resource_path,
            robot_state_publisher_node,
            gz_sim,
            spawn_entity,
            bridge_imu,  # Adding the bridge as a process
            bridge_front_lidar,
        ]
    )

