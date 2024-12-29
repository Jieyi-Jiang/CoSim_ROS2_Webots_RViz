import os
import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_dir = get_package_share_directory('my_package')
    robot_description_path = os.path.join(package_dir, 'resource', 'my_robot.urdf')
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    use_slam_toolbox = LaunchConfiguration('slam_toolbox', default=False)

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'my_world.wbt'),
        mode='realtime' #'pause'
    )

    my_robot_driver = WebotsController(
        robot_name='my_robot',
        parameters=[
            {'robot_description': robot_description_path,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True
             },
        ]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': '<robot name=""><link name=""/></robot>'}
        ],
    )

    # footprint_publisher = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     output='screen',
    #     arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
    # )
    toolbox_params = os.path.join(package_dir, 'resource', 'mapper_params_online_async.yaml')
    slam_toolbox = Node(
        parameters=[toolbox_params,
                    {'use_sim_time': use_sim_time}],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        condition=launch.conditions.IfCondition(use_slam_toolbox)
    )
    return LaunchDescription([
        webots,
        my_robot_driver,
        robot_state_publisher,
        # footprint_publisher,
        slam_toolbox,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])