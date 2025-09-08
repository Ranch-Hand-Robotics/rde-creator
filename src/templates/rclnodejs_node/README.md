# {{package_name}}

A ROS 2 Node.js node using rclnodejs with various communication patterns.

## Features

{{#if include_publisher}}
- **Publisher**: Publishes messages on topic `{{topic_name}}`
{{/if}}
{{#if include_subscriber}}
- **Subscriber**: Subscribes to messages on topic `{{topic_name}}`
{{/if}}
{{#if include_service}}
- **Service Server**: Provides service `{{service_name}}`
{{/if}}
{{#if include_client}}
- **Service Client**: Calls service `{{service_name}}`
{{/if}}
{{#if include_timer}}
- **Timer**: Executes periodic tasks every {{timer_period}} seconds
{{/if}}
{{#if include_parameters}}
- **Parameters**: Dynamic parameter handling
{{/if}}

## Prerequisites

- ROS 2 (Humble or later recommended)
- Node.js 16.0.0 or later
- rclnodejs

## Installation

1. Install dependencies:
   ```bash
   npm install
   ```

2. Source your ROS 2 installation:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

3. Build the ROS 2 package:
   ```bash
   colcon build
   ```

## Usage

### Running the Node

```bash
# From the package directory
source install/setup.bash
node node.js
```

### Using the Launch File

```bash
# From the workspace directory
source install/setup.bash
ros2 launch {{package_name}} node.launch.py
```

## API

### {{node_name | pascalCase}}Node

Main class for the ROS 2 node.

#### Methods

{{#if include_publisher}}
- `publishMessage(message)` - Publish a string message
{{/if}}
{{#if include_client}}
- `callService(a, b)` - Call the add two ints service
{{/if}}
{{#if include_parameters}}
- `getParameter(name)` - Get parameter value
- `setParameter(name, value)` - Set parameter value
{{/if}}

## Configuration

The node can be configured through:

- ROS 2 parameters
- Environment variables
- Configuration files

## Development

### Running Tests

```bash
npm test
```

### Development Mode

```bash
npm run dev
```

## ROS 2 Integration

This node integrates with ROS 2 through rclnodejs, providing:

- Full ROS 2 node lifecycle management
- Support for all ROS 2 message types
- Service client/server functionality
- Parameter handling
- QoS settings

## License

MIT