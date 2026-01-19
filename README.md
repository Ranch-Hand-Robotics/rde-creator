# ROS 2 Template Creator Extension

An AI-powered Visual Studio Code extension for creating ROS 2 packages from intelligent templates. This extension leverages GitHub Copilot and Language Models to generate complete, functional ROS 2 packages in C++, Python, and Node.js.

![Creator](https://raw.githubusercontent.com/Ranch-Hand-Robotics/rde-creator/refs/heads/main/media/screenshot.png)

## Features

### 🤖 AI-Powered Package Generation
- **Natural Language Descriptions**: Describe your ROS 2 node in plain English and let AI generate the complete implementation
- **Template-Based Creation**: Choose from pre-built templates for common ROS 2 patterns
- **Multi-Language Support**: Generate packages in C++, Python, and Node.js
- **Intelligent Code Completion**: Templates include GitHub Copilot prompts for enhanced development

### 📦 Available Templates

#### C++ ROS 2 Node ⚡
High-performance, real-time ROS 2 composable nodes with:
- Modern C++17+ standards
- Proper visibility control for Windows/Linux compatibility
- Component-based architecture with `rclcpp_components`
- Optional lifecycle management
- Comprehensive error handling and logging

#### Python ROS 2 Node 🐍
Fast prototyping with rich libraries:
- Modern Python 3.12+ syntax
- Async/await patterns for concurrent operations
- Comprehensive docstrings and type hints
- Built-in test generation with pytest

#### Node.js ROS 2 Node 🌐
Web integration and async I/O:
- Full rclnodejs integration
- Promise-based asynchronous operations
- Web-friendly development patterns
- REST API integration capabilities

#### Resource Package 📁
Configuration and asset management:
- URDF files for robot descriptions
- Launch configurations
- Mesh files and assets
- Configuration files and parameters

## Quick Start

### Installation
1. Install the extension from the VS Code marketplace
2. Ensure you have ROS 2 installed and sourced
3. Install GitHub Copilot Chat extension for AI-powered generation

### Creating Your First Package

1. **Open Command Palette** (`Ctrl+Shift+P` / `Cmd+Shift+P`)
2. **Run Command**: "ROS 2: Create Package"
3. **Choose Template**: Select from C++, Python, Node.js, or Resource templates
4. **Configure Options**: Set package metadata and features
5. **AI Generation** (Optional): Describe your node functionality in natural language
6. **Generate**: The extension creates a complete, buildable ROS 2 package

### Alternative: Context Menu Creation
- **Right-click** in the Explorer panel on any folder
- **Select**: "Create ROS 2 Package Here"
- **Follow the same configuration steps**

## Template Details

### C++ Template Features
- **Composable Nodes**: Full `rclcpp_components` support
- **Visibility Control**: Cross-platform symbol visibility management
- **Lifecycle Support**: Optional `rclcpp_lifecycle` integration
- **Modern CMake**: Automatic ROS distribution detection
- **Testing**: Integrated gtest framework with proper discovery

### Python Template Features
- **Async Support**: Native asyncio integration
- **Type Hints**: Full type annotation support
- **Testing**: pytest integration with ROS 2 fixtures
- **Documentation**: Auto-generated docstrings

### Node.js Template Features
- **Promise-Based**: Modern async/await patterns
- **Web Integration**: HTTP server capabilities
- **Testing**: Jest integration with ROS 2 mocks

## Development Workflow

1. **Generate Package**: Use the extension to create your initial package structure
2. **Customize**: Modify generated code to fit your specific requirements
3. **Build & Test**: Use standard ROS 2 tools (`colcon build`, `colcon test`)
4. **Iterate**: Leverage GitHub Copilot for code enhancements and debugging

## Requirements

- **VS Code**: Latest version recommended (1.101.0+)
- **ROS 2**: Humble, Iron, Jazzy, or Kilted
- **GitHub Copilot CLI** (**Required**): For AI-powered package generation
  - Install with: `npm install -g @github/copilot`
  - The extension will offer to install it for you if not found
  - Extension will not function without the CLI

## AI Generation

This extension uses the GitHub Copilot SDK for background agent functionality. This provides:
- Robust session management
- Better error handling
- Improved streaming response processing
- Official SDK support from GitHub

**Installation**: The extension will automatically check for the GitHub Copilot CLI when you try to create a package. If not installed, you'll be prompted to install it.

You can configure the CLI path in extension settings:
- `rosPackageCreator.copilotCLIPath`: Custom path to Copilot CLI (leave empty for default)

## Troubleshooting

### Common Issues

**"GitHub Copilot CLI is required"**
- Choose "Install Now" when prompted to automatically install the CLI
- Or manually install: `npm install -g @github/copilot`
- Verify the CLI is accessible in your PATH
- After installation, try creating a package again

**"Failed to initialize Copilot SDK"**
- Ensure the GitHub Copilot CLI is installed: `npm install -g @github/copilot`
- Verify the CLI is accessible in your PATH (run `copilot --version` in terminal)
- Check extension logs in the "RDE Creator" output channel
- If you have a custom CLI path, set it in `rosPackageCreator.copilotCLIPath`

**"Template generation failed"**
- Verify ROS 2 environment is properly sourced
- Check that target directory is writable
- Ensure template manifests are valid
- Check for error details in the "RDE Creator" output channel

**"Build failures after generation"**
- Verify all ROS 2 dependencies are installed
- Check that generated CMakeLists.txt matches your ROS distribution
- Ensure visibility control headers are properly configured

## License

MIT License - see [LICENSE](LICENSE) for details