import os
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir = get_package_share_directory('webots_robotnik')
    robot_description_path = os.path.join(package_dir, 'resource', 'rbrobout_controller.urdf')

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'Prueba_rbrobout_webots.wbt')
    )

    my_robot_driver = WebotsController(
        robot_name='rbrobout',
        parameters=[
            {'robot_description': robot_description_path},
        ]
    )

    return LaunchDescription([
        webots,
        my_robot_driver,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
