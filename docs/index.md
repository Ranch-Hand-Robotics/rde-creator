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

## User Guide
### Package Configuration

#### Basic Settings
- **Package Name**: ROS 2 package identifier
- **Node Name**: Primary node class/function name
- **Description**: Package purpose and functionality
- **License**: SPDX license identifier

#### Template Selection
Choose from available templates based on your needs:

- **C++**: High-performance, real-time applications
- **Python**: Rapid prototyping, rich ecosystem
- **Node.js**: Web integration, async operations
- **Resource**: Configuration files, URDF, launch files

#### AI Enhancement
For AI-powered generation:
- Describe your node's functionality in natural language
- Specify communication patterns (publishers, subscribers, services)
- Include performance requirements or constraints
- Mention any specific ROS 2 features needed

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

## Template Reference

### C++ ROS 2 Node
High-performance composable nodes with modern C++ standards.

**Features:**
- C++17+ with RAII principles
- Component-based architecture
- Cross-platform visibility control
- Optional lifecycle management
- Integrated gtest framework

**Use Cases:**
- Real-time control systems
- High-throughput data processing
- Performance-critical applications
- Hardware interface nodes

[View C++ Template Details](cpp-node.md)

### Python ROS 2 Node
Rapid prototyping with extensive libraries and async support.

**Features:**
- Python 3.12+ with type hints
- Async/await patterns
- Comprehensive docstrings
- pytest integration
- Rich ecosystem integration

**Use Cases:**
- Algorithm development
- Data analysis and visualization
- Integration with ML/AI frameworks
- Research and experimentation

[View Python Template Details](python-node.md)

### Node.js ROS 2 Node
Web-enabled nodes with promise-based async operations.

**Features:**
- Full rclnodejs integration
- Promise/async-await patterns
- Web server capabilities
- REST API integration
- Jest testing framework

**Use Cases:**
- Web-based ROS 2 interfaces
- IoT device integration
- Cloud service connectivity
- Real-time web dashboards

[View Node.js Template Details](nodejs-node.md)

### Resource Package
Configuration and asset management for ROS 2 packages.

**Features:**
- URDF robot descriptions
- Launch file generation
- Mesh and asset management
- Configuration file templates
- Parameter files

**Use Cases:**
- Robot description packages
- Launch configuration management
- Asset organization
- Parameter management

[View Resource Template Details](resource-package.md)

### Planned work
**Rust ROS 2 Node**
- Safe, concurrent systems programming
- Modern Rust features
- Integration with ROS 2 via rclrs
- Testing with Rust's built-in framework

**Micro-ROS Node**
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

## Advanced Usage

### Custom Templates
Create your own templates by following the [template development guide](../CONTRIBUTING.md#adding-new-templates).

### AI Prompt Engineering
For best AI generation results:
- Be specific about communication patterns
- Include performance requirements
- Specify message types and frequencies
- Mention error handling needs
- Describe testing requirements

### Integration with ROS 2 Tools
- **RViz**: Generated nodes include proper QoS settings
- **Gazebo**: Simulation-ready configurations
- **ros2cli**: Standard topic/service introspection
- **rosbag2**: Recording and playback compatibility

## Troubleshooting

### Common Issues

#### "Language Model API not available"
**Solution:** Install and activate GitHub Copilot Chat extension

#### "Build fails with visibility errors"
**Solution:** Check that `*_BUILDING_LIBRARY` macro is defined in CMakeLists.txt

#### "Python imports fail"
**Solution:** Ensure Python path includes workspace and check `setup.py` and `setup.cfg`

#### "Node doesn't register as component"
**Solution:** Verify `RCLCPP_COMPONENTS_REGISTER_NODE` macro usage

### Getting Help
- Check the [GitHub Issues](https://github.com/Ranch-Hand-Robotics/rde-creator/issues) page
- Review template-specific documentation
- Examine generated code comments for guidance

## Contributing
We welcome contributions! See our [contributing guide](../CONTRIBUTING.md) for details on:
- Adding new templates
- Improving existing templates
- Bug reports and feature requests
- Code contributions

## License
This extension is licensed under the MIT License. Template-generated code follows the license specified during package creation.






