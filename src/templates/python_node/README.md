# {{package_name}}

{{package_description}}

## Features

This ROS 2 Python node provides the following functionality:
{{#if include_publisher}}- **Publisher**: Publishes messages on topic `{{topic_name}}`
{{/if}}{{#if include_subscriber}}- **Subscriber**: Subscribes to messages from topic `{{topic_name}}`
{{/if}}{{#if include_service}}- **Service Server**: Provides service `{{service_name}}`
{{/if}}{{#if include_client}}- **Service Client**: Calls service `{{service_name}}`
{{/if}}{{#if include_timer}}- **Timer**: Executes periodic tasks every {{timer_period}} seconds
{{/if}}{{#if include_parameters}}- **Parameters**: Supports dynamic parameter configuration
{{/if}}

## Building

```bash
# Build the package
colcon build --packages-select {{package_name}}

# Source the workspace
source install/setup.bash
```

## Running

{{#if include_launch}}
### Using Launch File
```bash
ros2 launch {{package_name}} {{package_name}}.launch.py
```
{{/if}}

### Direct Execution
```bash
ros2 run {{package_name}} {{node_name}}
```

## Testing

```bash
# Run tests
colcon test --packages-select {{package_name}}
colcon test-result --verbose
```

## Node Parameters

{{#if include_parameters}}
- `example_param` (string, default: "default_value"): Example parameter for demonstration
{{/if}}

## Topics

{{#if include_publisher}}
- **Publisher**: `{{topic_name}}` (std_msgs/String)
{{/if}}{{#if include_subscriber}}
- **Subscriber**: `{{topic_name}}` (std_msgs/String)
{{/if}}

## Services

{{#if include_service}}
- **Server**: `{{service_name}}` (std_srvs/Trigger)
{{/if}}{{#if include_client}}
- **Client**: `{{service_name}}` (std_srvs/Trigger)
{{/if}}

## Architecture

This node follows ROS 2 best practices:
- Proper node lifecycle management
- Quality of Service (QoS) configuration for reliable communication
- Comprehensive error handling and logging
- Type hints for better code maintainability
- Unit tests for validation

## Dependencies

- `rclpy`: ROS 2 Python client library
- `std_msgs`: Standard ROS 2 message types
{{#if include_service}}- `std_srvs`: Standard ROS 2 service types
{{/if}}