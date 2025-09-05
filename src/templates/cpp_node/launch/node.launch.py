import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, ComposableNode, LoadComposableNodes
from launch_ros.descriptions import ComposableNodeDescription


def generate_launch_description():
    """
    Launch description for {{package_name}} C++ composable node.

    This launch file loads the {{node_name}} composable node into a container.
    {{#if include_lifecycle}}
    This is a lifecycle node that requires state management.
    {{/if}}
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

    # Create the composable node description
    node_description = ComposableNodeDescription(
        package='{{package_name}}',
        plugin='{{package_name}}::{{node_name|pascalcase}}',
        name='{{node_name}}',
        {{#if include_parameters}}
        parameters=[{
            'example_param': LaunchConfiguration('example_param')
        }],
        {{/if}}
    )

    # Create a container to load the composable node
    container = ComposableNodeContainer(
        name='{{package_name}}_container',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[node_description],
        output='screen',
    )

    {{#if include_lifecycle}}
    # For lifecycle nodes, we need to load and then manage the lifecycle
    load_node = LoadComposableNodes(
        composable_node_descriptions=[node_description],
        target_container='{{package_name}}_container',
    )

    # Create launch description with lifecycle management
    ld = LaunchDescription()

    {{#if include_parameters}}
    # Add launch arguments
    ld.add_action(example_param_arg)
    {{/if}}

    # Add container and load action
    ld.add_action(container)
    ld.add_action(load_node)

    {{else}}
    # Create launch description
    ld = LaunchDescription()

    {{#if include_parameters}}
    # Add launch arguments
    ld.add_action(example_param_arg)
    {{/if}}

    # Add the container with the composable node
    ld.add_action(container)
    {{/if}}

    return ld