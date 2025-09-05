import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch description for {{package_name}}.

    This launch file starts the {{node_name}} node with configurable parameters.
    """

    # Declare launch arguments
    {{#if include_parameters}}
    example_param_arg = DeclareLaunchArgument(
        'example_param',
        default_value='default_value',
        description='Example parameter for the node'
    )
    {{/if}}

    # Get package share directory
    pkg_share = get_package_share_directory('{{package_name}}')

    # Create the node
    node = Node(
        package='{{package_name}}',
        executable='{{node_name}}',
        name='{{node_name}}',
        output='screen',
        {{#if include_parameters}}
        parameters=[{
            'example_param': LaunchConfiguration('example_param')
        }],
        {{/if}}
    )

    # Create launch description
    ld = LaunchDescription()

    {{#if include_parameters}}
    # Add launch arguments
    ld.add_action(example_param_arg)
    {{/if}}

    # Add the node
    ld.add_action(node)

    return ld