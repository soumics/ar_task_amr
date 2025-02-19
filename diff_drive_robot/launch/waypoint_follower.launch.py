import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            output='screen'
        ),
    ])
