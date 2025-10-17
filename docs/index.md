# Robot Creator Extension Documentation

Welcome to the Creator for Robot Developer Extensions! 
This extension provides AI-powered tools to streamline ROS 2 package development across multiple programming languages. This extension works with your existing Copilot subscription with prompts specifically crafted for specific ROS 2 development scenarios.

> NOTE: AI Generated code may not always be correct or optimal. Always review and test generated code thoroughly. If you have issues or improvements for the templates, please submit an issue or pull request on GitHub.

## Getting Started

### Installation
1. Open VS Code
2. Go to Extensions (`Ctrl+Shift+X` / `Cmd+Shift+X`)
3. Search for "RDE ROS 2 Creator"
4. Click Install

### Requirements
- **ROS 2**: Humble, Iron, Jazzy, or Kilted
- **VS Code**: Latest version (1.101.0 or higher)
- **GitHub Copilot**: Required for AI features

### Quick Start
1. Open a ROS 2 workspace in VS Code
2. Right-click on your ROS 2 workspace's `src` directory in Explorer → "Create ROS 2 Package"
3. Choose your template and configure options
4. Use natural language to describe your node (optional)
5. Generate and build your package

Having issues? Check our [troubleshooting guide](troubleshooting.md).

### Building and Testing
#### Robot Developer Extensions for ROS 2
Select `ctrl+shift+b` and run `Colcon: Colcon Build Debug` to build your workspace.

#### Standard ROS 2 Workflow
```bash
# Source ROS 2
source /opt/ros/kilted/setup.bash

# Build the package
colcon build --packages-select your_package

# Source the workspace
source install/setup.bash

# Run tests
colcon test --packages-select your_package
colcon test-result --verbose
```

### Planned work

**Rust ROS 2 Package**

- Safe, concurrent systems programming
- Modern Rust features
- Integration with ROS 2 via rclrs
- Testing with Rust's built-in framework

**Micro-ROS Package**

- Embedded systems support
- Real-time capabilities
- Integration with micro-ROS libraries
- Lightweight communication patterns

**Simulation Package**

- Gazebo integration
- RViz configurations
- Simulation-ready launch files
- Sensor and actuator plugins
- Testing with simulation scenarios

**Dockerized Package**

- Containerized ROS 2 environments
- Dockerfile templates
- Docker Compose configurations
- CI/CD pipeline examples

## Discussions
[Github Discussions](https://github.com/orgs/Ranch-Hand-Robotics/discussions) are provided for community driven general guidance, walkthroughs, or support.

## Sponsor
If you find this extension useful, please consider [sponsoring Ranch Hand Robotics](https://github.com/sponsors/Ranch-Hand-Robotics) to help support the development of this extension and other open source projects.

## License
This extension is licensed under the MIT License. Template-generated code follows the license specified during package creation.






