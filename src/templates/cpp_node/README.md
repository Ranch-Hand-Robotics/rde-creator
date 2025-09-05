# {{package_name}}

{{package_description}}

## Features

This ROS 2 C++ composable node provides the following functionality:
{{#if include_publisher}}- **Publisher**: Publishes messages on topic `{{topic_name}}`
{{/if}}{{#if include_subscriber}}- **Subscriber**: Subscribes to messages from topic `{{topic_name}}`
{{/if}}{{#if include_service}}- **Service Server**: Provides service `{{service_name}}`
{{/if}}{{#if include_client}}- **Service Client**: Calls service `{{service_name}}`
{{/if}}{{#if include_timer}}- **Timer**: Executes periodic tasks every {{timer_period}} seconds
{{/if}}{{#if include_parameters}}- **Parameters**: Supports dynamic parameter configuration
{{/if}}{{#if include_lifecycle}}- **Lifecycle Management**: Supports lifecycle state transitions
{{/if}}

## Architecture

This is a **composable node** that can be loaded into a component container at runtime. {{#if include_lifecycle}}It also supports **lifecycle management** with proper state transitions.{{/if}}

### Composable Node Benefits
- **Runtime Loading**: Can be loaded/unloaded without restarting the process
- **Resource Efficiency**: Shares process resources with other components
- **Modularity**: Enables building complex systems from smaller components
- **Hot Swapping**: Components can be replaced without system downtime

{{#if include_lifecycle}}
### Lifecycle States
- **Unconfigured**: Node is created but not configured
- **Inactive**: Node is configured but not active
- **Active**: Node is fully operational
- **Finalized**: Node is shutting down
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

### Manual Component Loading
```bash
# Start a component container
ros2 run rclcpp_components component_container

# In another terminal, load the component
ros2 component load /ComponentManager {{package_name}} {{package_name}}::{{node_name|pascalcase}}
```

{{#if include_lifecycle}}
### Lifecycle Management
```bash
# Configure the node
ros2 lifecycle set /{{node_name}} configure

# Activate the node
ros2 lifecycle set /{{node_name}} activate

# Deactivate the node
ros2 lifecycle set /{{node_name}} deactivate

# Cleanup the node
ros2 lifecycle set /{{node_name}} cleanup
```
{{/if}}

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

## Component Information

- **Plugin Name**: `{{package_name}}::{{node_name|pascalcase}}`
- **Library**: `{{node_name}}_component`
- **Node Type**: {{#if include_lifecycle}}Lifecycle {{/if}}Composable Node

## Dependencies

- `rclcpp`: ROS 2 C++ client library
- `rclcpp_components`: Composable node support
- `std_msgs`: Standard ROS 2 message types
{{#if include_lifecycle}}- `rclcpp_lifecycle`: Lifecycle node support
{{/if}}{{#if include_service}}- `std_srvs`: Standard ROS 2 service types
{{/if}}

## Code Structure

```
{{package_name}}/
├── include/{{package_name}}/
│   └── {{node_name}}.hpp          # Class declaration
├── src/
│   └── {{node_name}}.cpp          # Implementation
├── CMakeLists.txt                 # Build configuration
├── package.xml                    # Package metadata
└── launch/
    └── {{package_name}}.launch.py # Launch file
```

## Development Notes

### Modern C++ Features
- Uses C++17 features (auto, smart pointers, lambdas)
- RAII principles for resource management
- Const correctness and noexcept where appropriate

### ROS 2 Best Practices
- Proper QoS settings for reliable communication
- Comprehensive error handling and logging
- Thread-safe operations
- Clean shutdown procedures

### Component Registration
The node is registered as a composable component using:
```cpp
RCLCPP_COMPONENTS_REGISTER_NODE({{package_name}}::{{node_name|pascalcase}})
```

This allows the node to be loaded dynamically at runtime using the component manager.